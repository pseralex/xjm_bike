#include "para/BikeParameters.hpp"
#include "ross/RosInterface.h"
#include "fed/RobotFeed.h"
#include "ref/RefTrajes.h"
#include "wbc/qpSolve/QpWbc.h"
#include "wbc/dataPlot/DataPlot.h"

using namespace std;

int main(int argc, char **argv)
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "dyn_pkg_node");

    BikeParameters m_conf;
    RosInterface m_rosInterface(m_conf);
    RobotFeed m_robotFeed(m_conf, m_rosInterface);
    RefTrajes m_refTrajes(m_conf);
    QpWbc m_qpWbc(m_conf,m_refTrajes);
    Vector_h m_tau_cmd = Vector_h::Zero(m_conf.m_nvi);
    Vector_h m_tau_qp = Vector_h::Zero(m_conf.m_ntau);
    DataPlot m_dataPlot;
    
    // Matrix33_h Rb = m_robotFeed.m_dyn_sol.yrp2rot(0.5,-0.001,0.001);
    // Eigen::Quaterniond q(Rb);
    // cout << m_robotFeed.m_dyn_sol.quat2yrp(q.x(),q.y(),q.z(),q.w()).transpose() << endl;

    ros::Rate loop_rate(1/m_conf.m_dt);
    // double m_time = 0.0;
    int m_count = 0;
    while(ros::ok()){
        // auto startTime = steady_clock::now();
        m_robotFeed.run(m_conf, m_rosInterface);
        m_refTrajes.perform_motion(m_conf, m_count);
        m_qpWbc.run(m_conf, m_robotFeed, m_refTrajes, m_count);
        m_tau_qp = m_qpWbc.get_tau_qp();
        m_tau_cmd << m_tau_qp(0), 0, m_tau_qp(1);
        
        // if (m_count < 4000){m_tau_cmd << -5*m_robotFeed.x_feedback(3),0,5;}
        // else {m_tau_cmd << -5*m_robotFeed.x_feedback(3),0,0;}
        
        m_rosInterface.tau_command_publish(m_tau_cmd);

        m_dataPlot.prepare_data(m_conf, m_count, m_refTrajes, m_robotFeed, m_qpWbc);

        m_count ++;
        // cout << m_count << ":" << m_qpWbc.m_q_all_ddot.transpose() << "..." << m_tau_cmd.transpose() << endl;
        if(m_count == m_conf.m_Num){break;}
        ros::spinOnce();  
        loop_rate.sleep();
        // auto endTime    = steady_clock::now();  
        // double duration = microseconds_duration(endTime-startTime).count();
        // // // // m_time += duration*1.0e-6; /// 时间单位为秒
        // cout << duration << endl;
    } /// while(ros::ok())
    cout << "====== end =======" << endl;

    m_dataPlot.plot_data(m_conf);

    while(ros::ok()){
        ros::spinOnce();  
        loop_rate.sleep();
    }
    return 0;
} 