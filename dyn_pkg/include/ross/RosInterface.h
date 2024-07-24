#ifndef ROSINTERFACE_H
#define ROSINTERFACE_H
#pragma once

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include "para/BikeParameters.hpp"

namespace wbc_controller_xjm{

  class RosInterface{
    
    public:
      RosInterface(const BikeParameters& conf);
      ~RosInterface();
      ros::NodeHandle m_nh; 

        /// *** Start ============================ 状态反馈 ============================ Start *** ///
        /// 浮动基-状态反馈
        void gazebo_basePosVel_state_Subscriber_CB(const nav_msgs::Odometry& basePosVel_msg);
        ros::Subscriber m_gazebo_basePosVel_state_Subscriber; 
        Eigen::VectorXd m_q_base_feedback;
        Eigen::VectorXd m_v_base_feedback;

        /// 关节-状态反馈
        ros::Subscriber m_gazebo_joints_state_Subscriber;  
        void gazebo_joints_state_Subscriber_CB(const sensor_msgs::JointState& joints_msg);
        Eigen::VectorXd m_q_joints_feedback;       
        Eigen::VectorXd m_v_joints_feedback;  

        /// *** End ============================ 状态反馈 ============================ End *** ///

        /// *** Start ============================ 指令下发 ============================ Start *** ///
        void tau_data_prepare(const Eigen::VectorXd& tau_joints); 
        void tau_command_publish(const Eigen::VectorXd& tau_joints); 

        /// 电机-力矩发布器&消息
        ros::Publisher    m_Publisher_tau_rear_joint;
        ros::Publisher    m_Publisher_tau_fork_joint;
        ros::Publisher    m_Publisher_tau_front_joint;

        std_msgs::Float64 m_msg_tau_rear_joint;
        std_msgs::Float64 m_msg_tau_fork_joint;
        std_msgs::Float64 m_msg_tau_front_joint;
        /// *** End ============================ 指令下发 ============================ End *** ///
    // private:       
  };
} 
#endif