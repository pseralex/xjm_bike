#include "ross/RosInterface.h"

namespace wbc_controller_xjm{

    /// 构造函数
    RosInterface::RosInterface(const BikeParameters& conf)
    {
        /// *** Start ============================ 状态反馈 ============================ Start *** ///
        /// 浮动基状态反馈
        m_gazebo_basePosVel_state_Subscriber = m_nh.subscribe("/bicycle_model/base_link_pos_vel", 10, &RosInterface::gazebo_basePosVel_state_Subscriber_CB, this);
        m_q_base_feedback.resize(7); m_q_base_feedback = conf.m_q0.head(7);
        m_v_base_feedback.resize(6); m_v_base_feedback = conf.m_v0.head(6);

        /// 关节状态反馈
        m_gazebo_joints_state_Subscriber = m_nh.subscribe("/bicycle_model/joint_states", 10, &RosInterface::gazebo_joints_state_Subscriber_CB, this);
        m_q_joints_feedback.resize(conf.m_na); m_q_joints_feedback = conf.m_q0.tail(conf.m_na);
        m_v_joints_feedback.resize(conf.m_na); m_v_joints_feedback = conf.m_v0.tail(conf.m_na);
        /// *** End ============================ 状态反馈 ============================ End *** ///
        
        /// *** Start ============================ 指令下发 ============================ Start *** ///
        /// 电机-发布器初始化
        m_Publisher_tau_rear_joint  = m_nh.advertise<std_msgs::Float64>("/bicycle_model/rear_joint_effort_controller/command", 10, this);
        m_Publisher_tau_fork_joint  = m_nh.advertise<std_msgs::Float64>("/bicycle_model/fork_joint_effort_controller/command", 10, this);
        m_Publisher_tau_front_joint = m_nh.advertise<std_msgs::Float64>("/bicycle_model/front_joint_effort_controller/command", 10, this);
        /// *** End ============================ 指令下发 ============================ End *** ///
    }
    RosInterface::~RosInterface(){}

    /// 浮动基状态反馈
    void RosInterface::gazebo_basePosVel_state_Subscriber_CB(const nav_msgs::Odometry& basePosVel_msg){

        /// 浮动基位姿(先位置，再姿态)
        m_q_base_feedback 
        << basePosVel_msg.pose.pose.position.x,
           basePosVel_msg.pose.pose.position.y,
           basePosVel_msg.pose.pose.position.z,
           basePosVel_msg.pose.pose.orientation.x, 
           basePosVel_msg.pose.pose.orientation.y,
           basePosVel_msg.pose.pose.orientation.z,
           basePosVel_msg.pose.pose.orientation.w;

        /// 浮动基速度(先线速度，再角速度)
        m_v_base_feedback
        << basePosVel_msg.twist.twist.linear.x,
           basePosVel_msg.twist.twist.linear.y,
           basePosVel_msg.twist.twist.linear.z,
           basePosVel_msg.twist.twist.angular.x,
           basePosVel_msg.twist.twist.angular.y,
           basePosVel_msg.twist.twist.angular.z;
    }

    /// 关节状态反馈
    void RosInterface::gazebo_joints_state_Subscriber_CB(const sensor_msgs::JointState& joints_msg){
    /// ======================================================================================== ///

        // -----------------------------------ros话题顺序-----------------------------------

        // cout << joints_msg.name[0] << "..." << joints_msg.name[1] << "..." << joints_msg.name[2] << endl;

        // 00- fork_joint                      
        // 01- front_joint
        // 02- rear_joint

         /// ======================================================================================== ///

        m_q_joints_feedback << joints_msg.position[0], joints_msg.position[1], joints_msg.position[2];
        m_v_joints_feedback << joints_msg.velocity[0], joints_msg.velocity[1], joints_msg.velocity[2];
    }

    /// 准备关节力矩数据
    void RosInterface::tau_data_prepare(const Eigen::VectorXd& tau_joints){
        m_msg_tau_fork_joint.data = tau_joints[0]; 
        m_msg_tau_front_joint.data = tau_joints[1]; 
        m_msg_tau_rear_joint.data = tau_joints[2];
    }

    /// 发布关节力矩
    void RosInterface::tau_command_publish(const Eigen::VectorXd& tau_joints){
        tau_data_prepare(tau_joints);
        m_Publisher_tau_fork_joint.publish(m_msg_tau_fork_joint);
        m_Publisher_tau_front_joint.publish(m_msg_tau_front_joint);
        m_Publisher_tau_rear_joint.publish(m_msg_tau_rear_joint);
    }
}

