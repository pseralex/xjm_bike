#ifndef BikeParameters_H
#define BikeParameters_H
#pragma once

#include "wbc/fwd/fwd.hpp"
#include <yaml-cpp/yaml.h>
#include <vector>
#include <math.h>

using namespace std;
using namespace wbc_controller_xjm;

namespace wbc_controller_xjm
{
    class BikeParameters{

    public:
    /// ------------------------------------成员变量------------------------------------///
        YAML::Node yaml_data;
        string m_path_ws_src;
        string m_urdf_path;
        
        /// ========================== 仿真设置 ========================== ///
        double m_dt;
        int    m_Num;
        bool   m_is_plot_data;
        string m_data_name;
        /// ========================== 仿真设置 ========================== ///

        /// ========================== 机器人基本信息 ========================== ///
        int m_nq;
        int m_nv;
        int m_na;
        int m_nvi;
        int m_ntau;
        int m_x_dim;
        int m_ntask;
        // double m_mass;

        Vector_h m_q0;
        Vector_h m_v0;
        double m_Robot_Initial_Height;
        /// ========================== 机器人基本信息 ========================== ///

        /// ========================== 控制参数 ========================== ///
        Vector_h m_kp; 
        Vector_h m_kd; 
        Matrix_h m_weight; 
        Vector_h m_ddq_tau_upper;
        Vector_h m_ddq_tau_lower;
        Matrix_h m_decision_variable_amp_limit;
        /// ========================== 控制参数 ========================== ///


    /// ------------------------------------构造函数------------------------------------///
    BikeParameters(){ 
        path_init();
        robot_info();               
        simulationSetting(); 
        taskall();
        decision_variable_amp_limit();
    };

    /// ------------------------------------成员函数------------------------------------///

    /// -------------------------------------------------------------------------------------------------- ///
    void path_init(){
        char path[PATH_MAX];
        ssize_t count = readlink("/proc/self/exe", path, PATH_MAX);
        std::string currentDir(path, (count > 0) ? count : 0);
        std::string::size_type pos = currentDir.find_last_of('/');
        currentDir = currentDir.substr(0, pos);
        m_path_ws_src = currentDir + "/../../../src";
        const std::string yamlDir = m_path_ws_src + "/dyn_pkg/include/para/Bicycle_Parameters.yaml"; 
        yaml_data = YAML::LoadFile(yamlDir);
        m_urdf_path = m_path_ws_src + "/bicycle_model/urdf/bicycle_model_pino.urdf"; 
    }

    /// -------------------------------------------------------------------------------------------------- ///
    void simulationSetting(){
        m_dt  = 0.001;
        m_Num = yaml_data["Num"].as<int>();
        m_data_name = yaml_data["data_name"].as<string>();
        m_is_plot_data = yaml_data["is_plot_data"].as<bool>();
    }

    /// -------------------------------------------------------------------------------------------------- ///
    void robot_info(){
        m_nq = 10;
        m_nv = 9;
        m_na = 3;
        m_nvi = 3;
        m_ntau = 2;
        m_x_dim = 3+2;
        m_ntask = 6;

        // m_mass = 11.7349;  /// kg

        /// ### 初始配置(须与URDF保持一致)
        m_q0 = Vector_h::Zero(m_nq);
        m_v0 = Vector_h::Zero(m_nv);
        m_Robot_Initial_Height  = yaml_data["Robot_Initial_Height"].as<double>();
        
        m_q0 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0; 
        m_q0[2] = m_Robot_Initial_Height; /// bicycle 初始高度  
    }

    void taskall(){
       
        m_kp = Vector_h::Zero(m_ntask);
        m_kd = Vector_h::Zero(m_ntask);
        m_weight = Matrix_h::Zero(m_ntask,m_ntask);
        Vector_h weight_diag = Vector_h::Zero(m_ntask);
        
        m_ddq_tau_upper = Vector_h::Zero(m_x_dim);
        m_ddq_tau_lower = Vector_h::Zero(m_x_dim);

        for(int i=0; i<m_ntask; i++){
            m_kp[i] = yaml_data["KP"][i].as<double>();
            m_kd[i] = yaml_data["KD"][i].as<double>();
            weight_diag[i] = sqrt(yaml_data["WEIGHT"][i].as<double>());
        }
        
        m_weight.diagonal() = weight_diag;
        
        for(int i=0; i<5; i++){
            m_ddq_tau_upper[i] = yaml_data["ddq_tau_upper"][i].as<double>();
            m_ddq_tau_lower[i] = yaml_data["ddq_tau_lower"][i].as<double>();
        }
        
    }

    void decision_variable_amp_limit(){
        m_decision_variable_amp_limit = Matrix_h::Zero(m_x_dim, m_x_dim);
        Vector_h decision_variable_amp_limit = yaml_data["w_ampLimit"].as<double>()*Vector_h::Ones(m_x_dim);
        m_decision_variable_amp_limit.diagonal() = decision_variable_amp_limit;
    }

    }; 
}
#endif