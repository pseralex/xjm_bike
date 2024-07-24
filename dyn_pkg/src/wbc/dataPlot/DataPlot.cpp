#include "wbc/dataPlot/DataPlot.h"

namespace wbc_controller_xjm
{
    DataPlot::DataPlot()
    {

    }
    DataPlot::~DataPlot(){}


    /// ================================== 准备数据 ================================== ///
    void DataPlot::prepare_data(const BikeParameters& conf, 
                                const int& count, 
                                const RefTrajes& refTrajes, 
                                const RobotFeed& robotFeed,
                                const QpWbc& qpWbc){

        if(conf.m_is_plot_data==true){
            prepare_data_time(conf, count);
            prepare_data_traking(conf, refTrajes, robotFeed);
            prepare_data_jnt_torque_acc(qpWbc);
        }                            
    }

    /// ================================== 绘图 ================================== ///
    void DataPlot::plot_data(const BikeParameters& conf){
        if(conf.m_is_plot_data==true){
            subplot_data_traking();
            subplot_data_jnt_torque_acc();
        }
    }

    /// ========================================================================= ///
    /// ========================================================================= ///
    /// ========================================================================= ///
    /// 准备时间数据
    void DataPlot::prepare_data_time(const BikeParameters& conf, const int& count){
        m_time.push_back(count*conf.m_dt); 
        m_count = count;
    }

    void DataPlot::prepare_data_traking(const BikeParameters& conf, const RefTrajes& refTrajes, const RobotFeed& robotFeed){

        Vector_h X_f = robotFeed.m_dyn_sol.m_tj.X_f;
        m_theta.push_back(X_f(0));
        m_x.push_back(X_f(1));
        m_y.push_back(X_f(2));
        m_z.push_back(X_f(3));
        m_phi.push_back(X_f(4));
        m_delta.push_back(X_f(5));
        
        Vector_h X_ref = refTrajes.m_tra_ref;
        m_theta_ref.push_back(X_ref(0));
        m_x_ref.push_back(X_ref(1));
        m_y_ref.push_back(X_ref(2));
        m_z_ref.push_back(X_ref(3));
        m_phi_ref.push_back(X_ref(4));
        m_delta_ref.push_back(X_ref(5));

    }

    void DataPlot::subplot_data_traking(){
        plt::suptitle("Trajectory Tracking: solid ref, Dotted fed");

        plt::subplot(2, 3, 1);
        plt::plot(
            m_time, m_theta_ref, "r", m_time, m_theta, "r--"
        );
        plt::subplot(2, 3, 2);
        plt::plot(
            m_time, m_x_ref, "g", m_time, m_x, "g--"
        );
        plt::subplot(2, 3, 3);
        plt::plot(
            m_time, m_y_ref, "b", m_time, m_y, "b--"
        );
        plt::subplot(2, 3, 4);
        plt::plot(
            m_time, m_z_ref, "c", m_time, m_z, "c--"
        );
        plt::subplot(2, 3, 5);
        plt::plot(
            m_time, m_phi_ref, "m", m_time, m_phi, "m--"
        );
        plt::subplot(2, 3, 6);
        plt::plot(
            m_time, m_delta_ref, "y", m_time, m_delta, "y--"  
        );

        plt::show(); 
    }

    void DataPlot::prepare_data_jnt_torque_acc(const QpWbc& qpWbc){
        m_delta_torque.push_back(qpWbc.m_tau_qp(0));
        m_phi_r_torque.push_back(qpWbc.m_tau_qp(1));
        m_theta_acc.push_back(qpWbc.m_q_all_ddot(0));
        m_delta_acc.push_back(qpWbc.m_q_all_ddot(1));
        m_phi_r_acc.push_back(qpWbc.m_q_all_ddot(2));
    }

    void DataPlot::subplot_data_jnt_torque_acc(){
        plt::suptitle("Joints torque and acc");

        plt::subplot(1, 2, 1);
        plt::plot(m_time, m_delta_torque, "r--", 
                  m_time, m_phi_r_torque, "g--");

        plt::subplot(1, 2, 2);
        plt::plot(m_time, m_theta_acc, "r--", 
                  m_time, m_delta_acc, "g--", 
                  m_time, m_phi_r_acc, "b--");

        plt::show();     
    }

}
