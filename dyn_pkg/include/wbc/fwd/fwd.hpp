#ifndef QpWbc_fwd_H
#define QpWbc_fwd_H
#pragma once

#include <Eigen/Core>
#include <chrono>
#include <iostream>
#include <string>
/// ================================= 头文件 ================================= ///
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
/// ================================= 头文件 ================================= ///

namespace wbc_controller_xjm
{

    using namespace std;
    using namespace pinocchio;

    typedef std::chrono::steady_clock                              steady_clock;
    typedef std::chrono::duration <double, std::nano>              nanoseconds_duration;  // 纳秒  
    typedef std::chrono::duration <double, std::micro>             microseconds_duration; // 微秒  
    typedef std::chrono::duration <double, std::milli>             milliseconds_duration; // 毫秒  
    typedef std::chrono::duration <double>                         seconds_duration;      // 秒  
    
    typedef std::size_t                                            Index_h;
    typedef Eigen::Matrix<double,Eigen::Dynamic,1>                 Vector_h;
    typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>    Matrix_h;
    typedef Eigen::VectorXi                                        VectorXi_h;
    typedef Eigen::Matrix<bool,Eigen::Dynamic,1>                   VectorXb_h; 
    typedef Eigen::Matrix<double,3,1>                              Vector3_h;
    typedef Eigen::Matrix<double,6,1>                              Vector6_h;
    typedef Eigen::Matrix<double,3,Eigen::Dynamic>                 Matrix3x_h;
    typedef Eigen::Matrix<double,3,3>                              Matrix33_h;
    typedef Eigen::Matrix<double,4,4>                              Matrix44_h;
    typedef Eigen::Matrix<double,6,Eigen::Dynamic>                 Matrix6x_h;
    typedef Eigen::Ref<Vector3_h>                                  RefVector3_h;
    typedef const Eigen::Ref<const Vector3_h>                      ConstRefVector3_h;
    typedef Eigen::Ref<Vector_h>                                   RefVector_h;
    typedef const Eigen::Ref<const Vector_h>                       ConstRefVector_h;
    typedef Eigen::Ref<Matrix_h>                                   RefMatrix_h;
    typedef const Eigen::Ref<const Matrix_h>                       ConstRefMatrix_h;
    typedef Eigen::Map<const Eigen::Matrix<double, 3, 3>>          MapMatrix33_h;

    typedef pinocchio::Model        Model;
    typedef pinocchio::Data         Data;
    typedef pinocchio::SE3          SE3;
    typedef pinocchio::Motion       Motion;
    typedef pinocchio::Frame        Frame;          
}

#endif