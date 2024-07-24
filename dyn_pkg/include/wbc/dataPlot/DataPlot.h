#ifndef DATAPLOT_H
#define DATAPLOT_H

#pragma once

#include "matplotlibcpp.h"
#include <iostream>
#include <cmath>
#include <string>
#include "para/BikeParameters.hpp"
#include "fed/RobotFeed.h"
#include "ref/RefTrajes.h"
#include "wbc/qpSolve/QpWbc.h"

namespace plt = matplotlibcpp;

///  ============================ 用颜色表示顺序  ============================ ///
/// 01:r=红色; 02:g=绿; 03:b=蓝色; 04:c=青绿色; 05:m=洋红色;  06:y=黄色; 07:k=黑色;
///  ============================ 用颜色表示顺序  ============================ ///
namespace wbc_controller_xjm
{
    class DataPlot
    {
    public:
        DataPlot();
        ~DataPlot();

        /// ====================================== 准备数据 ====================================== ///
        void prepare_data(const BikeParameters& conf, 
                          const int& count, 
                          const RefTrajes& refTrajes, 
                          const RobotFeed& robotFeed,
                          const QpWbc& qpWbc);
        void plot_data(const BikeParameters& conf);

        double m_count;
        std::vector<double> m_time;
        void prepare_data_time(const BikeParameters& conf, const int& count);

        std::vector<double> m_theta;
        std::vector<double> m_x;
        std::vector<double> m_y;
        std::vector<double> m_z;
        std::vector<double> m_phi;
        std::vector<double> m_delta;
        std::vector<double> m_theta_ref;
        std::vector<double> m_x_ref;
        std::vector<double> m_y_ref;
        std::vector<double> m_z_ref;
        std::vector<double> m_phi_ref;
        std::vector<double> m_delta_ref;

        void prepare_data_traking(const BikeParameters& conf, const RefTrajes& refTrajes, const RobotFeed& robotFeed);
        void subplot_data_traking();

        std::vector<double> m_delta_torque;
        std::vector<double> m_phi_r_torque;
        std::vector<double> m_theta_acc;
        std::vector<double> m_delta_acc;
        std::vector<double> m_phi_r_acc;
        
        void prepare_data_jnt_torque_acc(const QpWbc& qpWbc);
        void subplot_data_jnt_torque_acc();

    // private:
    };
}
#endif