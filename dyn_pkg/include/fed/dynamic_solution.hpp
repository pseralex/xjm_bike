/**
 * @file dynamic_solution.hpp
 * @author xiongjiaming (xiongjiaming@pku.edu.cn)
 * @brief 自行车动力学解算库
 * @version 1.0
 * @date 2024-05-08
 * 
 * @copyright Copyright (c) 2024
 * 
 */

#ifndef DYNAMICSOLUTION_H
#define DYNAMICSOLUTION_H
#pragma once

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <ros/package.h>

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/jacobian.hpp"

#include "pinocchio/algorithm/crba.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/algorithm/centroidal.hpp"
#include <pinocchio/algorithm/frames.hpp>

#include "pinocchio/algorithm/kinematics-derivatives.hpp"

using namespace pinocchio;

const double m_DEFAULT_EP = 1e-12;
const int m_DEFAULT_ITMAX = 200;
const double MAX_VALUE = 1e100;
const double m_pi = 3.14159265358979323846;

struct vel_cons
{
    Eigen::Matrix<double, 9, 3> X, dX;   
    Eigen::Matrix<double, 6, 1> dq_d;    // dx dy dz dpsi dphi dphi_f
};

struct dyn_matrix
{   
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> M9;   // 9*9质量矩阵
    Eigen::Matrix<double, Eigen::Dynamic, 1> C9;   // 9*1科氏力和重力项
    Eigen::Matrix<double, 3, 3> M;   // 3*3质量矩阵
    Eigen::Matrix<double, 3, 1> C;    // 3*1科氏力和重力项
    Eigen::Matrix<double, 3, 2> S;        
};

struct task_jacobi
{   
    Eigen::Matrix<double, Eigen::Dynamic, 1> X_f;    // theta, x, y, z, phi, delta
    Eigen::Matrix<double, Eigen::Dynamic, 3> JX;
    Eigen::Matrix<double, Eigen::Dynamic, 3> dJX;
};

class m_model
{
public:
    m_model();
    Model model;    
};

class DynamicSolution
{
public:
    DynamicSolution();
    m_model md;
    Data data;
    dyn_matrix m_dm;
    task_jacobi m_tj;
    vel_cons m_vel;
    Eigen::Matrix<double, 6, 2> m_geo;
    double m_total;    
    Eigen::Matrix<double, 6, 1> m_solution_0;
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> m_Jqv, m_dJqv;
    Eigen::Matrix<double, 3, 3> m_Rb, m_Bw, m_dBw;
    double ep_0;
    int it_max_0;
    
    /**
     * @brief 接触几何关系求解
     * 给定theta,delta，求z,varphi,xi,p4,eta,p8
     * @param x=[theta,delta]
     * @param x0=[z,varphi,xi,p4,eta,p8]的初始值
     * @param ep 误差阈值
     * @param it_max 最大迭代次数
     * @return Eigen::Matrix<double, 6, 1> xsolution=[z,varphi,xi,p4,eta,p8]
     */
    Eigen::Matrix<double, 6, 1> for_kin(Eigen::Vector2d x, Eigen::Matrix<double, 6, 1> x0, double ep, int it_max);

    /**
     * @brief 速度约束Jacobi求解
     * 
     * @param x=[psi theta phi delta xi p4 eta p8]
     * @param dx=[dtheta ddelta dphi_r]
     */
    void Vel_Jacobian(Eigen::Matrix<double, 8, 1> x, Eigen::Vector3d dx);

    /**
     * @brief 求动力学矩阵：质量矩阵、科氏力和重力项、接触Jacobi矩阵及其关于时间导数
     * 
     * @param x=[x,y,psi,theta,delta]
     * @param v=[dtheta,ddelta,dphi_r]
     * @param dyn_matrix 输出变量，见该结构体注释
     */
    void dynamic_matrices(Eigen::Matrix<double, 5, 1> x, Eigen::Vector3d v); 

    Eigen::Matrix<double, 3, 3> yrp2rot(double psi, double theta, double phi);

    Eigen::Matrix<double, 3, 3> yrp2omega(double psi, double theta, double phi);

    Eigen::Matrix<double, 3, 3> d_yrp2omega(double psi, double theta, double phi, double dpsi, double dtheta, double dphi);

    Eigen::Vector3d quat2yrp(double qx, double qy, double qz, double qw);
    
private:
    Eigen::Matrix<double, 6, 6> PHI2(Eigen::Vector3d v);
    Eigen::Matrix<double, 6, 6> dPHI2(Eigen::Vector3d v);
    Eigen::Matrix<double, 3, 6> PHI2_lin(Eigen::Vector3d v);
    Eigen::Matrix<double, 3, 6> dPHI2_lin(Eigen::Vector3d v);
    double m_sign(double x);
    double w, c, lambda, R_r, R_f, R_w, xbr, zbr, xbd, zbd, xdf, x_B, z_B;
};

#endif