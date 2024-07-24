/**
 * @file dynamic_solution.cpp
 * @author xiongjiaming (xiongjiaming@pku.edu.cn)
 * @brief 自行车动力学解算库
 * @version 1.0
 * @date 2024-05-08
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "fed/dynamic_solution.hpp"
#include<limits>

using namespace Eigen;
using namespace std;
using namespace pinocchio;

static Matrix<double, 3, 3> hat(Vector3d v)
{
   Matrix<double, 3, 3> M1;
   M1 << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
   return M1;
}

m_model::m_model()
{
    std::string models_package_path = ros::package::getPath("bicycle_model");
    const std::string urdf_filename = models_package_path + "/urdf/bicycle_model_pino.urdf"; 
    pinocchio::urdf::buildModel(urdf_filename,pinocchio::JointModelFreeFlyer(),model);
}


DynamicSolution::DynamicSolution()
:data(md.model)
{  
   m_dm.M = MatrixXd::Zero(3, 3);
   m_dm.C = MatrixXd::Zero(3, 1);
   m_dm.M9 = MatrixXd::Zero(9, 9);
   m_dm.C9 = MatrixXd::Zero(9, 1);
   m_dm.S = MatrixXd::Zero(3, 2);

   m_dm.S(1,0) = 1;
   m_dm.S(2,1) = 1;

   m_tj.X_f = MatrixXd::Zero(6, 1);
   m_tj.JX = MatrixXd::Zero(6, 3);
   m_tj.dJX = MatrixXd::Zero(6, 3);

   m_vel.X = MatrixXd::Zero(9, 3);
   m_vel.dX = MatrixXd::Zero(9, 3);
   m_vel.dq_d = MatrixXd::Zero(6, 1);
   m_geo = MatrixXd::Zero(6, 2);
   m_vel.X.row(4) << 1,0,0;
   m_vel.X.row(6) << 0,1,0;
   m_vel.X.row(8) << 0,0,1;

   m_Jqv = MatrixXd::Zero(9, 9);
   m_dJqv = MatrixXd::Zero(9, 9);
   m_Jqv.block<3,3>(6,6) = Matrix3d::Identity();

   m_Rb = Matrix3d::Identity();
   m_Bw = MatrixXd::Zero(3, 3);
   m_dBw = MatrixXd::Zero(3, 3);

   ep_0 = 1e-12; 
   it_max_0 = 200; 
   m_total=pinocchio::computeTotalMass(md.model);

   w=0.35836; c=0.01477408; lambda=0.162705; R_r=0.1-0.005; R_f=0.09-0.005; R_w=0.005; x_B=0.19196755; z_B=0.29996175;
   // w=0.35836; c=0.01477408; lambda=0.162705; R_r=0.1-0.005; R_f=0.09-0.005; R_w=0.005; x_B=0.080325; z_B=0.33209;
   xbr=-x_B; zbr=R_r+R_w-z_B;
   xbd=(c*cos(lambda)-(R_f+R_w)*sin(lambda))*cos(lambda)+w-x_B;
   zbd=(c*cos(lambda)-(R_f+R_w)*sin(lambda))*sin(lambda)+(R_f+R_w)-z_B;
   xdf=(R_f+R_w)*sin(lambda)-c*cos(lambda);

   m_solution_0 << z_B, 0, m_pi/2, 0, m_pi/2+lambda, 0;
}


double DynamicSolution::m_sign(double x)
{
    return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
}

Matrix<double, 6, 6> DynamicSolution::PHI2(Vector3d v)
{
   MatrixXd M1 = MatrixXd::Zero(6, 6);
   M1.block<3, 3>(0, 0) = Matrix3d::Identity();
   M1.block<3, 3>(3, 3) = Matrix3d::Identity();
   M1.block<3, 3>(0, 3) = -hat(v);
   return M1;
}

Matrix<double, 6, 6> DynamicSolution::dPHI2(Vector3d v)
{
   MatrixXd M1 = MatrixXd::Zero(6, 6);
   M1.block<3, 3>(0, 3) = -hat(v);
   return M1;
}

Matrix<double, 3, 6> DynamicSolution::PHI2_lin(Vector3d v)
{
   MatrixXd M1 = MatrixXd::Zero(3, 6);
   M1.block<3, 3>(0, 0) = Matrix3d::Identity();
   M1.block<3, 3>(0, 3) = -hat(v);
   return M1;
}

Matrix<double, 3, 6> DynamicSolution::dPHI2_lin(Vector3d v)
{
   MatrixXd M1 = MatrixXd::Zero(3, 6);
   M1.block<3, 3>(0, 3) = -hat(v);
   return M1;
}

Matrix<double, 3, 3> DynamicSolution::yrp2rot(double psi, double theta, double phi)
{
   Matrix<double, 3, 3> R;
   R(0,0) = cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta);
   R(0,1) = -cos(theta)*sin(psi);
   R(0,2) = cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta);
   R(1,0) = cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta);
   R(1,1) = cos(psi)*cos(theta);
   R(1,2) = sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta);
   R(2,0) = -cos(theta)*sin(phi);
   R(2,1) = sin(theta);
   R(2,2) = cos(phi)*cos(theta);
   return R;
}

Matrix<double, 3, 3> DynamicSolution::yrp2omega(double psi, double theta, double phi)
{
   Matrix<double, 3, 3> Bw;
   Bw(0,0) = -cos(theta)*sin(phi);
   Bw(0,1) = cos(phi);
   Bw(0,2) = 0;
   Bw(1,0) = sin(theta);
   Bw(1,1) = 0;
   Bw(1,2) = 1;
   Bw(2,0) = cos(phi)*cos(theta);
   Bw(2,1) = sin(phi);
   Bw(2,2) = 0;
   return Bw;
}

Matrix<double, 3, 3> DynamicSolution::d_yrp2omega(double psi, double theta, double phi, double dpsi, double dtheta, double dphi)
{
   Matrix<double, 3, 3> dBw;
   dBw(0,0) = dtheta*sin(phi)*sin(theta) - dphi*cos(phi)*cos(theta);
   dBw(0,1) = -dphi*sin(phi);
   dBw(0,2) = 0;
   dBw(1,0) = dtheta*cos(theta);
   dBw(1,1) = 0;
   dBw(1,2) = 0;
   dBw(2,0) = - dphi*cos(theta)*sin(phi) - dtheta*cos(phi)*sin(theta);
   dBw(2,1) = dphi*cos(phi);
   dBw(2,2) = 0;
   return dBw;
}

Vector3d DynamicSolution::quat2yrp(double qx, double qy, double qz, double qw)
{
   // Quaterniond quat(w,x,y,z);
   // Vector3d yrp = quat.matrix().eulerAngles(2,0,1);

   double x = 0.0;
   double y = 0.0; 
   double z = 0.0; 
   double w = 1.0;
   double q_norm = sqrt(qx*qx+qy*qy+qz*qz+qw*qw);
   if(q_norm==1){
       x = qx;
       y = qy;
       z = qz;
       w = qw;
   }
   else{
       x = qx/q_norm;
       y = qy/q_norm;
       z = qz/q_norm;
       w = qw/q_norm;
   }
   
   Vector3d yrp =  Vector3d::Zero(3);

   double psi = atan2(-2*(x*y-z*w),1-2*x*x-2*z*z);
   double theta = asin(2*(y*z+x*w));
   double phi = atan2(-2*(x*z-y*w),1-2*x*x-2*y*y); 

   yrp << psi, theta, phi;

   return yrp;
}

Eigen::Matrix<double, 6, 1> DynamicSolution::for_kin(Eigen::Vector2d x, Eigen::Matrix<double, 6, 1> x0, double ep = m_DEFAULT_EP, int it_max = m_DEFAULT_ITMAX)
{
   double theta=x(0), delta=x(1);
   double z=x0(0), phi=x0(1), xi=x0(2), p4=x0(3), eta=x0(4), p8=x0(5);
   Matrix<double, 6, 1> f, d, xs, xsolution;
   xsolution << 0,0,0,0,0,0;
   Matrix<double, 6, 6> J;
   int k=1;
   while(k<it_max)
{  
   f(0) = cos(theta)*sin(phi)*(R_r*sin(xi) + R_w*cos(p4)*sin(xi)) - cos(phi)*cos(theta)*(R_r*cos(xi) + R_w*cos(p4)*cos(xi));
   f(1) = R_w*cos(p4)*sin(theta) + R_w*cos(phi)*cos(theta)*sin(p4)*sin(xi) + R_w*cos(theta)*cos(xi)*sin(p4)*sin(phi);
   f(2) = - (R_f*sin(eta) + R_w*cos(p8)*sin(eta))*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) - (R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(cos(lambda)*cos(phi)*cos(theta) + cos(theta)*sin(lambda)*sin(phi));
   f(3) = R_w*cos(p8)*(cos(delta)*sin(theta) + cos(lambda)*sin(delta)*cos(theta)*sin(phi) - cos(phi)*sin(delta)*cos(theta)*sin(lambda)) - R_w*cos(eta)*sin(p8)*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) + R_w*sin(eta)*sin(p8)*(cos(lambda)*cos(phi)*cos(theta) + cos(theta)*sin(lambda)*sin(phi));
   f(4) = z - cos(theta)*sin(phi)*(R_r*cos(xi) + R_w*cos(p4)*cos(xi)) - cos(phi)*cos(theta)*(R_r*sin(xi) + R_w*cos(p4)*sin(xi)) + R_w*sin(p4)*sin(theta) + zbr*cos(phi)*cos(theta) - xbr*cos(theta)*sin(phi);
   f(5) = z + (R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) + xdf*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) - (cos(lambda)*cos(phi)*cos(theta) + cos(theta)*sin(lambda)*sin(phi))*(R_f*sin(eta) + R_w*cos(p8)*sin(eta)) + zbd*cos(phi)*cos(theta) + R_w*sin(p8)*(cos(delta)*sin(theta) + cos(lambda)*sin(delta)*cos(theta)*sin(phi) - cos(phi)*sin(delta)*cos(theta)*sin(lambda)) - xbd*cos(theta)*sin(phi);
   J(0,0) = 0;
   J(0,1) = cos(theta)*sin(phi)*(R_r*cos(xi) + R_w*cos(p4)*cos(xi)) + cos(phi)*cos(theta)*(R_r*sin(xi) + R_w*cos(p4)*sin(xi));
   J(0,2) = cos(theta)*sin(phi)*(R_r*cos(xi) + R_w*cos(p4)*cos(xi)) + cos(phi)*cos(theta)*(R_r*sin(xi) + R_w*cos(p4)*sin(xi));
   J(0,3) = R_w*cos(phi)*cos(theta)*cos(xi)*sin(p4) - R_w*cos(theta)*sin(p4)*sin(phi)*sin(xi);
   J(0,4) = 0;
   J(0,5) = 0;
   J(1,0) = 0;
   J(1,1) = R_w*cos(phi)*cos(theta)*cos(xi)*sin(p4) - R_w*cos(theta)*sin(p4)*sin(phi)*sin(xi);
   J(1,2) = R_w*cos(phi)*cos(theta)*cos(xi)*sin(p4) - R_w*cos(theta)*sin(p4)*sin(phi)*sin(xi);
   J(1,3) = R_w*cos(p4)*cos(phi)*cos(theta)*sin(xi) - R_w*sin(p4)*sin(theta) + R_w*cos(p4)*cos(theta)*cos(xi)*sin(phi);
   J(1,4) = 0;
   J(1,5) = 0;
   J(2,0) = 0;
   J(2,1) = (R_f*sin(eta) + R_w*cos(p8)*sin(eta))*(cos(delta)*cos(lambda)*cos(phi)*cos(theta) + cos(delta)*cos(theta)*sin(lambda)*sin(phi)) + (R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(cos(lambda)*cos(theta)*sin(phi) - cos(phi)*cos(theta)*sin(lambda));
   J(2,2) = 0;
   J(2,3) = 0;
   J(2,4) = (cos(lambda)*cos(phi)*cos(theta) + cos(theta)*sin(lambda)*sin(phi))*(R_f*sin(eta) + R_w*cos(p8)*sin(eta)) - (R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda));
   J(2,5) = R_w*sin(eta)*sin(p8)*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) + R_w*cos(eta)*sin(p8)*(cos(lambda)*cos(phi)*cos(theta) + cos(theta)*sin(lambda)*sin(phi));
   J(3,0) = 0;
   J(3,1) = R_w*cos(p8)*(cos(lambda)*cos(phi)*sin(delta)*cos(theta) + sin(delta)*cos(theta)*sin(lambda)*sin(phi)) + R_w*cos(eta)*sin(p8)*(cos(delta)*cos(lambda)*cos(phi)*cos(theta) + cos(delta)*cos(theta)*sin(lambda)*sin(phi)) - R_w*sin(eta)*sin(p8)*(cos(lambda)*cos(theta)*sin(phi) - cos(phi)*cos(theta)*sin(lambda));
   J(3,2) = 0;
   J(3,3) = 0;
   J(3,4) = R_w*sin(eta)*sin(p8)*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) + R_w*cos(eta)*sin(p8)*(cos(lambda)*cos(phi)*cos(theta) + cos(theta)*sin(lambda)*sin(phi));
   J(3,5) = R_w*cos(p8)*sin(eta)*(cos(lambda)*cos(phi)*cos(theta) + cos(theta)*sin(lambda)*sin(phi)) - R_w*cos(eta)*cos(p8)*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) - R_w*sin(p8)*(cos(delta)*sin(theta) + cos(lambda)*sin(delta)*cos(theta)*sin(phi) - cos(phi)*sin(delta)*cos(theta)*sin(lambda));
   J(4,0) = 1;
   J(4,1) = cos(theta)*sin(phi)*(R_r*sin(xi) + R_w*cos(p4)*sin(xi)) - xbr*cos(phi)*cos(theta) - cos(phi)*cos(theta)*(R_r*cos(xi) + R_w*cos(p4)*cos(xi)) - zbr*cos(theta)*sin(phi);
   J(4,2) = cos(theta)*sin(phi)*(R_r*sin(xi) + R_w*cos(p4)*sin(xi)) - cos(phi)*cos(theta)*(R_r*cos(xi) + R_w*cos(p4)*cos(xi));
   J(4,3) = R_w*cos(p4)*sin(theta) + R_w*cos(phi)*cos(theta)*sin(p4)*sin(xi) + R_w*cos(theta)*cos(xi)*sin(p4)*sin(phi);
   J(4,4) = 0;
   J(4,5) = 0;
   J(5,0) = 1;
   J(5,1) = (cos(lambda)*cos(theta)*sin(phi) - cos(phi)*cos(theta)*sin(lambda))*(R_f*sin(eta) + R_w*cos(p8)*sin(eta)) - xdf*(cos(delta)*cos(lambda)*cos(phi)*cos(theta) + cos(delta)*cos(theta)*sin(lambda)*sin(phi)) - (R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(cos(delta)*cos(lambda)*cos(phi)*cos(theta) + cos(delta)*cos(theta)*sin(lambda)*sin(phi)) + R_w*sin(p8)*(cos(lambda)*cos(phi)*sin(delta)*cos(theta) + sin(delta)*cos(theta)*sin(lambda)*sin(phi)) - xbd*cos(phi)*cos(theta) - zbd*cos(theta)*sin(phi);
   J(5,2) = 0;
   J(5,3) = 0;
   J(5,4) = - (R_f*sin(eta) + R_w*cos(p8)*sin(eta))*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) - (R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(cos(lambda)*cos(phi)*cos(theta) + cos(theta)*sin(lambda)*sin(phi));
   J(5,5) = R_w*cos(p8)*(cos(delta)*sin(theta) + cos(lambda)*sin(delta)*cos(theta)*sin(phi) - cos(phi)*sin(delta)*cos(theta)*sin(lambda)) - R_w*cos(eta)*sin(p8)*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) + R_w*sin(eta)*sin(p8)*(cos(lambda)*cos(phi)*cos(theta) + cos(theta)*sin(lambda)*sin(phi));

   d=-J.inverse()*f;
   xs=x0+d;
if(d.norm()<ep)
{
   xsolution=xs;
   m_solution_0=xs;
   break;
}
   x0=xs;
   k++;
   z=x0(0); phi=x0(1); xi=x0(2); p4=x0(3); eta=x0(4); p8=x0(5);
}
   return xsolution;
}

void DynamicSolution::Vel_Jacobian(Matrix<double, 8, 1> x, Vector3d dx)
{
   double psi=x(0), theta=x(1), phi=x(2), delta=x(3), xi=x(4), p4=x(5), eta=x(6), p8=x(7);
   Matrix<double, 6, 9> A, dA;
   Matrix<double, 6, 3> Xd, dXd;
   Matrix<double, 6, 8> Jq;

   Jq(0,0) = cos(phi)*sin(theta)*(R_r*cos(xi) + R_w*cos(p4)*cos(xi)) - sin(phi)*sin(theta)*(R_r*sin(xi) + R_w*cos(p4)*sin(xi));
   Jq(0,1) = 0;
   Jq(0,2) = 0;
   Jq(0,3) = cos(theta)*sin(phi)*(R_r*cos(xi) + R_w*cos(p4)*cos(xi)) + cos(phi)*cos(theta)*(R_r*sin(xi) + R_w*cos(p4)*sin(xi));
   Jq(0,4) = cos(theta)*sin(phi)*(R_r*cos(xi) + R_w*cos(p4)*cos(xi)) + cos(phi)*cos(theta)*(R_r*sin(xi) + R_w*cos(p4)*sin(xi));
   Jq(0,5) = R_w*cos(phi)*cos(theta)*cos(xi)*sin(p4) - R_w*cos(theta)*sin(p4)*sin(phi)*sin(xi);
   Jq(0,6) = 0;
   Jq(0,7) = 0;
   Jq(1,0) = R_w*cos(p4)*cos(theta) - R_w*cos(phi)*sin(p4)*sin(theta)*sin(xi) - R_w*cos(xi)*sin(p4)*sin(phi)*sin(theta);
   Jq(1,1) = 0;
   Jq(1,2) = 0;
   Jq(1,3) = R_w*cos(phi)*cos(theta)*cos(xi)*sin(p4) - R_w*cos(theta)*sin(p4)*sin(phi)*sin(xi);
   Jq(1,4) = R_w*cos(phi)*cos(theta)*cos(xi)*sin(p4) - R_w*cos(theta)*sin(p4)*sin(phi)*sin(xi);
   Jq(1,5) = R_w*cos(p4)*cos(phi)*cos(theta)*sin(xi) - R_w*sin(p4)*sin(theta) + R_w*cos(p4)*cos(theta)*cos(xi)*sin(phi);
   Jq(1,6) = 0;
   Jq(1,7) = 0;
   Jq(2,0) = (R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(sin(lambda)*sin(phi)*sin(theta) + cos(lambda)*cos(phi)*sin(theta)) - (R_f*sin(eta) + R_w*cos(p8)*sin(eta))*(sin(delta)*cos(theta) + cos(delta)*cos(lambda)*sin(phi)*sin(theta) - cos(delta)*cos(phi)*sin(lambda)*sin(theta));
   Jq(2,1) = -(R_f*sin(eta) + R_w*cos(p8)*sin(eta))*(cos(delta)*sin(theta) + cos(lambda)*sin(delta)*cos(theta)*sin(phi) - cos(phi)*sin(delta)*cos(theta)*sin(lambda));
   Jq(2,2) = 0;
   Jq(2,3) = (R_f*sin(eta) + R_w*cos(p8)*sin(eta))*(cos(delta)*cos(lambda)*cos(phi)*cos(theta) + cos(delta)*cos(theta)*sin(lambda)*sin(phi)) + (R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(cos(lambda)*cos(theta)*sin(phi) - cos(phi)*cos(theta)*sin(lambda));
   Jq(2,4) = 0;
   Jq(2,5) = 0;
   Jq(2,6) = (cos(lambda)*cos(phi)*cos(theta) + cos(theta)*sin(lambda)*sin(phi))*(R_f*sin(eta) + R_w*cos(p8)*sin(eta)) - (R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda));
   Jq(2,7) = R_w*sin(eta)*sin(p8)*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) + R_w*cos(eta)*sin(p8)*(cos(lambda)*cos(phi)*cos(theta) + cos(theta)*sin(lambda)*sin(phi));
   Jq(3,0) = R_w*cos(p8)*(cos(delta)*cos(theta) - cos(lambda)*sin(delta)*sin(phi)*sin(theta) + cos(phi)*sin(delta)*sin(lambda)*sin(theta)) - R_w*cos(eta)*sin(p8)*(sin(delta)*cos(theta) + cos(delta)*cos(lambda)*sin(phi)*sin(theta) - cos(delta)*cos(phi)*sin(lambda)*sin(theta)) - R_w*sin(eta)*sin(p8)*(sin(lambda)*sin(phi)*sin(theta) + cos(lambda)*cos(phi)*sin(theta));
   Jq(3,1) = - R_w*cos(p8)*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) - R_w*cos(eta)*sin(p8)*(cos(delta)*sin(theta) + cos(lambda)*sin(delta)*cos(theta)*sin(phi) - cos(phi)*sin(delta)*cos(theta)*sin(lambda));
   Jq(3,2) = 0;
   Jq(3,3) = R_w*cos(p8)*(cos(lambda)*cos(phi)*sin(delta)*cos(theta) + sin(delta)*cos(theta)*sin(lambda)*sin(phi)) + R_w*cos(eta)*sin(p8)*(cos(delta)*cos(lambda)*cos(phi)*cos(theta) + cos(delta)*cos(theta)*sin(lambda)*sin(phi)) - R_w*sin(eta)*sin(p8)*(cos(lambda)*cos(theta)*sin(phi) - cos(phi)*cos(theta)*sin(lambda));
   Jq(3,4) = 0;
   Jq(3,5) = 0;
   Jq(3,6) = R_w*sin(eta)*sin(p8)*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) + R_w*cos(eta)*sin(p8)*(cos(lambda)*cos(phi)*cos(theta) + cos(theta)*sin(lambda)*sin(phi));
   Jq(3,7) = R_w*cos(p8)*sin(eta)*(cos(lambda)*cos(phi)*cos(theta) + cos(theta)*sin(lambda)*sin(phi)) - R_w*cos(eta)*cos(p8)*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) - R_w*sin(p8)*(cos(delta)*sin(theta) + cos(lambda)*sin(delta)*cos(theta)*sin(phi) - cos(phi)*sin(delta)*cos(theta)*sin(lambda));
   Jq(4,0) = R_w*cos(theta)*sin(p4) + sin(phi)*sin(theta)*(R_r*cos(xi) + R_w*cos(p4)*cos(xi)) + cos(phi)*sin(theta)*(R_r*sin(xi) + R_w*cos(p4)*sin(xi)) - zbr*cos(phi)*sin(theta) + xbr*sin(phi)*sin(theta);
   Jq(4,1) = 0;
   Jq(4,2) = 1;
   Jq(4,3) = cos(theta)*sin(phi)*(R_r*sin(xi) + R_w*cos(p4)*sin(xi)) - xbr*cos(phi)*cos(theta) - cos(phi)*cos(theta)*(R_r*cos(xi) + R_w*cos(p4)*cos(xi)) - zbr*cos(theta)*sin(phi);
   Jq(4,4) = cos(theta)*sin(phi)*(R_r*sin(xi) + R_w*cos(p4)*sin(xi)) - cos(phi)*cos(theta)*(R_r*cos(xi) + R_w*cos(p4)*cos(xi));
   Jq(4,5) = R_w*cos(p4)*sin(theta) + R_w*cos(phi)*cos(theta)*sin(p4)*sin(xi) + R_w*cos(theta)*cos(xi)*sin(p4)*sin(phi);
   Jq(4,6) = 0;
   Jq(4,7) = 0;
   Jq(5,0) = (R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(sin(delta)*cos(theta) + cos(delta)*cos(lambda)*sin(phi)*sin(theta) - cos(delta)*cos(phi)*sin(lambda)*sin(theta)) + xdf*(sin(delta)*cos(theta) + cos(delta)*cos(lambda)*sin(phi)*sin(theta) - cos(delta)*cos(phi)*sin(lambda)*sin(theta)) + (R_f*sin(eta) + R_w*cos(p8)*sin(eta))*(sin(lambda)*sin(phi)*sin(theta) + cos(lambda)*cos(phi)*sin(theta)) - zbd*cos(phi)*sin(theta) + R_w*sin(p8)*(cos(delta)*cos(theta) - cos(lambda)*sin(delta)*sin(phi)*sin(theta) + cos(phi)*sin(delta)*sin(lambda)*sin(theta)) + xbd*sin(phi)*sin(theta);
   Jq(5,1) = (R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(cos(delta)*sin(theta) + cos(lambda)*sin(delta)*cos(theta)*sin(phi) - cos(phi)*sin(delta)*cos(theta)*sin(lambda)) + xdf*(cos(delta)*sin(theta) + cos(lambda)*sin(delta)*cos(theta)*sin(phi) - cos(phi)*sin(delta)*cos(theta)*sin(lambda)) - R_w*sin(p8)*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda));
   Jq(5,2) = 1;
   Jq(5,3) = (cos(lambda)*cos(theta)*sin(phi) - cos(phi)*cos(theta)*sin(lambda))*(R_f*sin(eta) + R_w*cos(p8)*sin(eta)) - xdf*(cos(delta)*cos(lambda)*cos(phi)*cos(theta) + cos(delta)*cos(theta)*sin(lambda)*sin(phi)) - (R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(cos(delta)*cos(lambda)*cos(phi)*cos(theta) + cos(delta)*cos(theta)*sin(lambda)*sin(phi)) + R_w*sin(p8)*(cos(lambda)*cos(phi)*sin(delta)*cos(theta) + sin(delta)*cos(theta)*sin(lambda)*sin(phi)) - xbd*cos(phi)*cos(theta) - zbd*cos(theta)*sin(phi);
   Jq(5,4) = 0;
   Jq(5,5) = 0;
   Jq(5,6) = - (R_f*sin(eta) + R_w*cos(p8)*sin(eta))*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) - (R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(cos(lambda)*cos(phi)*cos(theta) + cos(theta)*sin(lambda)*sin(phi));
   Jq(5,7) = R_w*cos(p8)*(cos(delta)*sin(theta) + cos(lambda)*sin(delta)*cos(theta)*sin(phi) - cos(phi)*sin(delta)*cos(theta)*sin(lambda)) - R_w*cos(eta)*sin(p8)*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) + R_w*sin(eta)*sin(p8)*(cos(lambda)*cos(phi)*cos(theta) + cos(theta)*sin(lambda)*sin(phi));

   m_geo = -Jq.rightCols<6>().inverse()*Jq.leftCols<2>();
   
   A(0,0) = 1;
   A(0,1) = 0;
   A(0,2) = 0;
   A(0,3) = sin(xi)*(R_r + R_w*cos(p4))*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - zbr*sin(phi)*sin(psi) - cos(xi)*(R_r + R_w*cos(p4))*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - xbr*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - R_w*cos(psi)*cos(theta)*sin(p4) + zbr*cos(phi)*cos(psi)*sin(theta);
   A(0,4) = zbr*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) - xbr*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - cos(xi)*(R_r + R_w*cos(p4))*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(xi)*(R_r + R_w*cos(p4))*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta));
   A(0,5) = 0;
   A(0,6) = R_w*sin(p4)*sin(psi)*sin(theta) + zbr*cos(phi)*cos(theta)*sin(psi) - xbr*cos(theta)*sin(phi)*sin(psi) - cos(phi)*cos(theta)*sin(psi)*sin(xi)*(R_r + R_w*cos(p4)) - cos(theta)*cos(xi)*sin(phi)*sin(psi)*(R_r + R_w*cos(p4));
   A(0,7) = 0;
   A(0,8) = - (R_r*cos(xi) + R_w*cos(p4)*cos(xi))*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - (R_r*sin(xi) + R_w*cos(p4)*sin(xi))*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta));
   A(1,0) = 0;
   A(1,1) = 1;
   A(1,2) = 0;
   A(1,3) = xbr*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + zbr*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) + cos(xi)*(R_r + R_w*cos(p4))*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) - sin(xi)*(R_r + R_w*cos(p4))*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - R_w*cos(theta)*sin(p4)*sin(psi);
   A(1,4) = zbr*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - xbr*sin(phi)*sin(psi) - cos(xi)*(R_r + R_w*cos(p4))*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - sin(xi)*(R_r + R_w*cos(p4))*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + xbr*cos(phi)*cos(psi)*sin(theta);
   A(1,5) = 0;
   A(1,6) = xbr*cos(psi)*cos(theta)*sin(phi) - zbr*cos(phi)*cos(psi)*cos(theta) - R_w*cos(psi)*sin(p4)*sin(theta) + cos(phi)*cos(psi)*cos(theta)*sin(xi)*(R_r + R_w*cos(p4)) + cos(psi)*cos(theta)*cos(xi)*sin(phi)*(R_r + R_w*cos(p4));
   A(1,7) = 0;
   A(1,8) = - (R_r*cos(xi) + R_w*cos(p4)*cos(xi))*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - (R_r*sin(xi) + R_w*cos(p4)*sin(xi))*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta));
   A(2,0) = 0;
   A(2,1) = 0;
   A(2,2) = 1;
   A(2,3) = 0;
   A(2,4) = cos(theta)*sin(phi)*(R_r*sin(xi) + R_w*cos(p4)*sin(xi)) - xbr*cos(phi)*cos(theta) - cos(phi)*cos(theta)*(R_r*cos(xi) + R_w*cos(p4)*cos(xi)) - zbr*cos(theta)*sin(phi);
   A(2,5) = 0;
   A(2,6) = R_w*cos(theta)*sin(p4) - zbr*cos(phi)*sin(theta) + xbr*sin(phi)*sin(theta) + cos(phi)*sin(theta)*sin(xi)*(R_r + R_w*cos(p4)) + cos(xi)*sin(phi)*sin(theta)*(R_r + R_w*cos(p4));
   A(2,7) = 0;
   A(2,8) = -cos(phi + xi)*cos(theta)*(R_r + R_w*cos(p4));
   A(3,0) = 1;
   A(3,1) = 0;
   A(3,2) = 0;
   A(3,3) = (R_f*sin(eta) + R_w*cos(p8)*sin(eta))*(cos(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - sin(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta))) - xbd*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - zbd*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - xdf*(cos(delta)*cos(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + cos(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) + cos(psi)*sin(delta)*cos(theta)) - (R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(cos(delta)*cos(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + cos(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) + cos(psi)*sin(delta)*cos(theta)) + R_w*sin(p8)*(cos(lambda)*sin(delta)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + sin(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - cos(delta)*cos(psi)*cos(theta));
   A(3,4) = zbd*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) - xbd*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - xdf*(cos(delta)*cos(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - cos(delta)*sin(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta))) - sin(eta)*(R_f + R_w*cos(p8))*(cos(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta))) - cos(eta)*(R_f + R_w*cos(p8))*(cos(delta)*cos(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - cos(delta)*sin(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta))) + R_w*sin(p8)*(cos(lambda)*sin(delta)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(delta)*sin(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)));
   A(3,5) = - (R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(cos(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta))) - (R_f*sin(eta) + R_w*cos(p8)*sin(eta))*(cos(delta)*cos(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + cos(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(delta)*cos(theta)*sin(psi));
   A(3,6) = xdf*sin(psi)*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) + cos(eta)*sin(psi)*(R_f + R_w*cos(p8))*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) + zbd*cos(phi)*cos(theta)*sin(psi) + R_w*sin(p8)*sin(psi)*(cos(delta)*sin(theta) + cos(lambda)*sin(delta)*cos(theta)*sin(phi) - cos(phi)*sin(delta)*cos(theta)*sin(lambda)) - xbd*cos(theta)*sin(phi)*sin(psi) - cos(theta)*sin(eta)*sin(psi)*cos(lambda - phi)*(R_f + R_w*cos(p8));
   A(3,7) = - xdf*(cos(lambda)*sin(delta)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + sin(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) + cos(delta)*cos(theta)*sin(psi)) - cos(eta)*(R_f + R_w*cos(p8))*(cos(lambda)*sin(delta)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + sin(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) + cos(delta)*cos(theta)*sin(psi)) - R_w*sin(p8)*(cos(delta)*cos(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + cos(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(delta)*cos(theta)*sin(psi));
   A(3,8) = 0;
   A(4,0) = 0;
   A(4,1) = 1;
   A(4,2) = 0;
   A(4,3) = xdf*(cos(delta)*cos(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + cos(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(delta)*cos(theta)*sin(psi)) + xbd*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + zbd*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - (R_f*sin(eta) + R_w*cos(p8)*sin(eta))*(cos(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta))) + (R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(cos(delta)*cos(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + cos(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(delta)*cos(theta)*sin(psi)) - R_w*sin(p8)*(cos(lambda)*sin(delta)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + sin(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) + cos(delta)*cos(theta)*sin(psi));
   A(4,4) = zbd*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - xbd*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - xdf*(cos(delta)*cos(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - cos(delta)*sin(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta))) - (R_f*sin(eta) + R_w*cos(p8)*sin(eta))*(cos(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta))) - (R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(cos(delta)*cos(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - cos(delta)*sin(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta))) + R_w*sin(p8)*(cos(lambda)*sin(delta)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - sin(delta)*sin(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)));
   A(4,5) = - cos(eta)*(R_f + R_w*cos(p8))*(cos(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - sin(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta))) - sin(eta)*(R_f + R_w*cos(p8))*(cos(delta)*cos(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + cos(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) + cos(psi)*sin(delta)*cos(theta));
   A(4,6) = xbd*cos(psi)*cos(theta)*sin(phi) - cos(eta)*cos(psi)*(R_f + R_w*cos(p8))*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) - zbd*cos(phi)*cos(psi)*cos(theta) - R_w*cos(psi)*sin(p8)*(cos(delta)*sin(theta) + cos(lambda)*sin(delta)*cos(theta)*sin(phi) - cos(phi)*sin(delta)*cos(theta)*sin(lambda)) - xdf*cos(psi)*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) + cos(psi)*cos(theta)*sin(eta)*cos(lambda - phi)*(R_f + R_w*cos(p8));
   A(4,7) = - xdf*(cos(lambda)*sin(delta)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + sin(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - cos(delta)*cos(psi)*cos(theta)) - cos(eta)*(R_f + R_w*cos(p8))*(cos(lambda)*sin(delta)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + sin(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - cos(delta)*cos(psi)*cos(theta)) - R_w*sin(p8)*(cos(delta)*cos(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + cos(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) + cos(psi)*sin(delta)*cos(theta));
   A(4,8) = 0;
   A(5,0) = 0;
   A(5,1) = 0;
   A(5,2) = 1;
   A(5,3) = 0;
   A(5,4) = R_w*sin(delta)*cos(theta)*sin(p8)*cos(lambda - phi) - zbd*cos(theta)*sin(phi) - sin(lambda - phi)*cos(theta)*sin(eta)*(R_f + R_w*cos(p8)) - xdf*cos(delta)*cos(theta)*cos(lambda - phi) - xbd*cos(phi)*cos(theta) - cos(delta)*cos(eta)*cos(theta)*cos(lambda - phi)*(R_f + R_w*cos(p8));
   A(5,5) = - sin(eta)*(R_f + R_w*cos(p8))*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) - cos(eta)*cos(theta)*cos(lambda - phi)*(R_f + R_w*cos(p8));
   A(5,6) = xdf*(sin(delta)*cos(theta) + cos(delta)*cos(lambda)*sin(phi)*sin(theta) - cos(delta)*cos(phi)*sin(lambda)*sin(theta)) + cos(eta)*(R_f + R_w*cos(p8))*(sin(delta)*cos(theta) + cos(delta)*cos(lambda)*sin(phi)*sin(theta) - cos(delta)*cos(phi)*sin(lambda)*sin(theta)) - zbd*cos(phi)*sin(theta) + R_w*sin(p8)*(cos(delta)*cos(theta) - cos(lambda)*sin(delta)*sin(phi)*sin(theta) + cos(phi)*sin(delta)*sin(lambda)*sin(theta)) + xbd*sin(phi)*sin(theta) + sin(eta)*sin(theta)*cos(lambda - phi)*(R_f + R_w*cos(p8));
   A(5,7) = xdf*(cos(delta)*sin(theta) + cos(lambda)*sin(delta)*cos(theta)*sin(phi) - cos(phi)*sin(delta)*cos(theta)*sin(lambda)) + cos(eta)*(R_f + R_w*cos(p8))*(cos(delta)*sin(theta) + cos(lambda)*sin(delta)*cos(theta)*sin(phi) - cos(phi)*sin(delta)*cos(theta)*sin(lambda)) - R_w*sin(p8)*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda));
   A(5,8) = 0;

   Xd = -A.leftCols<6>().inverse()*A.rightCols<3>();
   m_vel.X.topRows<4>() = Xd.topRows<4>();
   m_vel.X.row(5) = Xd.row(4);
   m_vel.X.row(7) = Xd.row(5);
   
   Matrix<double, 6, 1> dpc = m_geo*dx.topRows<2>();
   Matrix<double, 6, 1> dq = Xd*dx;
   m_vel.dq_d = dq;

   double dpsi=dq(3), dtheta=dx(0), dphi=dq(4), ddelta=dx(1), dxi=dpc(2), dp4=dpc(3), deta=dpc(4), dp8=dpc(5);

   dA(0,0) = 0;
   dA(0,1) = 0;
   dA(0,2) = 0;
   dA(0,3) = dxi*(cos(xi)*(R_r + R_w*cos(p4))*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) + sin(xi)*(R_r + R_w*cos(p4))*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta))) - dp4*(R_w*cos(p4)*cos(psi)*cos(theta) - R_w*cos(xi)*sin(p4)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + R_w*sin(p4)*sin(xi)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta))) - dtheta*(xbr*cos(psi)*cos(theta)*sin(phi) - zbr*cos(phi)*cos(psi)*cos(theta) - R_w*cos(psi)*sin(p4)*sin(theta) + cos(phi)*cos(psi)*cos(theta)*sin(xi)*(R_r + R_w*cos(p4)) + cos(psi)*cos(theta)*cos(xi)*sin(phi)*(R_r + R_w*cos(p4))) + dphi*(xbr*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - zbr*cos(phi)*sin(psi) + cos(xi)*(R_r + R_w*cos(p4))*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) + sin(xi)*(R_r + R_w*cos(p4))*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - zbr*cos(psi)*sin(phi)*sin(theta)) - dpsi*(xbr*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + zbr*cos(psi)*sin(phi) + cos(xi)*(R_r + R_w*cos(p4))*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) - sin(xi)*(R_r + R_w*cos(p4))*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - R_w*cos(theta)*sin(p4)*sin(psi) + zbr*cos(phi)*sin(psi)*sin(theta));
   dA(0,4) = dp4*(R_w*cos(xi)*sin(p4)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) + R_w*sin(p4)*sin(xi)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta))) - dxi*(cos(xi)*(R_r + R_w*cos(p4))*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) - sin(xi)*(R_r + R_w*cos(p4))*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta))) - dtheta*(xbr*cos(phi)*cos(theta)*sin(psi) + zbr*cos(theta)*sin(phi)*sin(psi) + cos(phi)*cos(theta)*cos(xi)*sin(psi)*(R_r + R_w*cos(p4)) - cos(theta)*sin(phi)*sin(psi)*sin(xi)*(R_r + R_w*cos(p4))) - dphi*(xbr*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + zbr*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) + cos(xi)*(R_r + R_w*cos(p4))*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) - sin(xi)*(R_r + R_w*cos(p4))*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta))) + dpsi*(xbr*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - zbr*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + cos(xi)*(R_r + R_w*cos(p4))*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) + sin(xi)*(R_r + R_w*cos(p4))*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)));
   dA(0,5) = 0;
   dA(0,6) = dp4*(R_w*cos(p4)*sin(psi)*sin(theta) + R_w*cos(phi)*cos(theta)*sin(p4)*sin(psi)*sin(xi) + R_w*cos(theta)*cos(xi)*sin(p4)*sin(phi)*sin(psi)) - dpsi*(xbr*cos(psi)*cos(theta)*sin(phi) - zbr*cos(phi)*cos(psi)*cos(theta) - R_w*cos(psi)*sin(p4)*sin(theta) + cos(phi)*cos(psi)*cos(theta)*sin(xi)*(R_r + R_w*cos(p4)) + cos(psi)*cos(theta)*cos(xi)*sin(phi)*(R_r + R_w*cos(p4))) - dxi*(cos(phi)*cos(theta)*cos(xi)*sin(psi)*(R_r + R_w*cos(p4)) - cos(theta)*sin(phi)*sin(psi)*sin(xi)*(R_r + R_w*cos(p4))) - dphi*(xbr*cos(phi)*cos(theta)*sin(psi) + zbr*cos(theta)*sin(phi)*sin(psi) + cos(phi)*cos(theta)*cos(xi)*sin(psi)*(R_r + R_w*cos(p4)) - cos(theta)*sin(phi)*sin(psi)*sin(xi)*(R_r + R_w*cos(p4))) + dtheta*(R_w*cos(theta)*sin(p4)*sin(psi) - zbr*cos(phi)*sin(psi)*sin(theta) + xbr*sin(phi)*sin(psi)*sin(theta) + cos(phi)*sin(psi)*sin(theta)*sin(xi)*(R_r + R_w*cos(p4)) + cos(xi)*sin(phi)*sin(psi)*sin(theta)*(R_r + R_w*cos(p4)));
   dA(0,7) = 0;
   dA(0,8) = dp4*(R_w*cos(xi)*sin(p4)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) + R_w*sin(p4)*sin(xi)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta))) - dphi*((R_r*cos(xi) + R_w*cos(p4)*cos(xi))*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) - (R_r*sin(xi) + R_w*cos(p4)*sin(xi))*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta))) + dpsi*((R_r*cos(xi) + R_w*cos(p4)*cos(xi))*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) + (R_r*sin(xi) + R_w*cos(p4)*sin(xi))*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta))) - dxi*((R_r*cos(xi) + R_w*cos(p4)*cos(xi))*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) - (R_r*sin(xi) + R_w*cos(p4)*sin(xi))*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta))) - dtheta*(cos(phi)*cos(theta)*sin(psi)*(R_r*cos(xi) + R_w*cos(p4)*cos(xi)) - cos(theta)*sin(phi)*sin(psi)*(R_r*sin(xi) + R_w*cos(p4)*sin(xi)));
   dA(1,0) = 0;
   dA(1,1) = 0;
   dA(1,2) = 0;
   dA(1,3) = - dxi*(cos(xi)*(R_r + R_w*cos(p4))*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) + sin(xi)*(R_r + R_w*cos(p4))*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta))) - dp4*(R_w*cos(p4)*cos(theta)*sin(psi) + R_w*cos(xi)*sin(p4)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) - R_w*sin(p4)*sin(xi)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta))) - dtheta*(xbr*cos(theta)*sin(phi)*sin(psi) - zbr*cos(phi)*cos(theta)*sin(psi) - R_w*sin(p4)*sin(psi)*sin(theta) + cos(phi)*cos(theta)*sin(psi)*sin(xi)*(R_r + R_w*cos(p4)) + cos(theta)*cos(xi)*sin(phi)*sin(psi)*(R_r + R_w*cos(p4))) - dphi*(xbr*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - zbr*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + cos(xi)*(R_r + R_w*cos(p4))*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) + sin(xi)*(R_r + R_w*cos(p4))*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta))) - dpsi*(xbr*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + zbr*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) + cos(xi)*(R_r + R_w*cos(p4))*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - sin(xi)*(R_r + R_w*cos(p4))*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) + R_w*cos(psi)*cos(theta)*sin(p4));
   dA(1,4) = dp4*(R_w*cos(xi)*sin(p4)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) + R_w*sin(p4)*sin(xi)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta))) - dxi*(cos(xi)*(R_r + R_w*cos(p4))*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - sin(xi)*(R_r + R_w*cos(p4))*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta))) - dphi*(zbr*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) + xbr*cos(phi)*sin(psi) + cos(xi)*(R_r + R_w*cos(p4))*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - sin(xi)*(R_r + R_w*cos(p4))*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) + xbr*cos(psi)*sin(phi)*sin(theta)) - dpsi*(xbr*cos(psi)*sin(phi) - zbr*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + cos(xi)*(R_r + R_w*cos(p4))*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) + sin(xi)*(R_r + R_w*cos(p4))*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + xbr*cos(phi)*sin(psi)*sin(theta)) + dtheta*(xbr*cos(phi)*cos(psi)*cos(theta) + zbr*cos(psi)*cos(theta)*sin(phi) - cos(psi)*cos(theta)*sin(phi)*sin(xi)*(R_r + R_w*cos(p4)) + cos(phi)*cos(psi)*cos(theta)*cos(xi)*(R_r + R_w*cos(p4)));
   dA(1,5) = 0;
   dA(1,6) = dphi*(xbr*cos(phi)*cos(psi)*cos(theta) + zbr*cos(psi)*cos(theta)*sin(phi) - cos(psi)*cos(theta)*sin(phi)*sin(xi)*(R_r + R_w*cos(p4)) + cos(phi)*cos(psi)*cos(theta)*cos(xi)*(R_r + R_w*cos(p4))) - dp4*(R_w*cos(p4)*cos(psi)*sin(theta) + R_w*cos(phi)*cos(psi)*cos(theta)*sin(p4)*sin(xi) + R_w*cos(psi)*cos(theta)*cos(xi)*sin(p4)*sin(phi)) - dtheta*(R_w*cos(psi)*cos(theta)*sin(p4) - zbr*cos(phi)*cos(psi)*sin(theta) + xbr*cos(psi)*sin(phi)*sin(theta) + cos(phi)*cos(psi)*sin(theta)*sin(xi)*(R_r + R_w*cos(p4)) + cos(psi)*cos(xi)*sin(phi)*sin(theta)*(R_r + R_w*cos(p4))) - dpsi*(xbr*cos(theta)*sin(phi)*sin(psi) - zbr*cos(phi)*cos(theta)*sin(psi) - R_w*sin(p4)*sin(psi)*sin(theta) + cos(phi)*cos(theta)*sin(psi)*sin(xi)*(R_r + R_w*cos(p4)) + cos(theta)*cos(xi)*sin(phi)*sin(psi)*(R_r + R_w*cos(p4))) - dxi*(cos(psi)*cos(theta)*sin(phi)*sin(xi)*(R_r + R_w*cos(p4)) - cos(phi)*cos(psi)*cos(theta)*cos(xi)*(R_r + R_w*cos(p4)));
   dA(1,7) = 0;
   dA(1,8) = dp4*(R_w*cos(xi)*sin(p4)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) + R_w*sin(p4)*sin(xi)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta))) - dphi*((R_r*cos(xi) + R_w*cos(p4)*cos(xi))*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - (R_r*sin(xi) + R_w*cos(p4)*sin(xi))*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta))) - dpsi*((R_r*cos(xi) + R_w*cos(p4)*cos(xi))*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) + (R_r*sin(xi) + R_w*cos(p4)*sin(xi))*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta))) - dxi*((R_r*cos(xi) + R_w*cos(p4)*cos(xi))*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - (R_r*sin(xi) + R_w*cos(p4)*sin(xi))*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta))) + dtheta*(cos(phi)*cos(psi)*cos(theta)*(R_r*cos(xi) + R_w*cos(p4)*cos(xi)) - cos(psi)*cos(theta)*sin(phi)*(R_r*sin(xi) + R_w*cos(p4)*sin(xi)));
   dA(2,0) = 0;
   dA(2,1) = 0;
   dA(2,2) = 0;
   dA(2,3) = 0;
   dA(2,4) = dxi*(cos(theta)*sin(phi)*(R_r*cos(xi) + R_w*cos(p4)*cos(xi)) + cos(phi)*cos(theta)*(R_r*sin(xi) + R_w*cos(p4)*sin(xi))) + dphi*(cos(theta)*sin(phi)*(R_r*cos(xi) + R_w*cos(p4)*cos(xi)) + cos(phi)*cos(theta)*(R_r*sin(xi) + R_w*cos(p4)*sin(xi)) - zbr*cos(phi)*cos(theta) + xbr*cos(theta)*sin(phi)) + dtheta*(cos(phi)*sin(theta)*(R_r*cos(xi) + R_w*cos(p4)*cos(xi)) + xbr*cos(phi)*sin(theta) - sin(phi)*sin(theta)*(R_r*sin(xi) + R_w*cos(p4)*sin(xi)) + zbr*sin(phi)*sin(theta)) + dp4*(R_w*cos(phi)*cos(theta)*cos(xi)*sin(p4) - R_w*cos(theta)*sin(p4)*sin(phi)*sin(xi));
   dA(2,5) = 0;
   dA(2,6) = dtheta*(xbr*cos(theta)*sin(phi) - zbr*cos(phi)*cos(theta) - R_w*sin(p4)*sin(theta) + cos(phi)*cos(theta)*sin(xi)*(R_r + R_w*cos(p4)) + cos(theta)*cos(xi)*sin(phi)*(R_r + R_w*cos(p4))) + dxi*(cos(phi)*cos(xi)*sin(theta)*(R_r + R_w*cos(p4)) - sin(phi)*sin(theta)*sin(xi)*(R_r + R_w*cos(p4))) + dphi*(xbr*cos(phi)*sin(theta) + zbr*sin(phi)*sin(theta) + cos(phi)*cos(xi)*sin(theta)*(R_r + R_w*cos(p4)) - sin(phi)*sin(theta)*sin(xi)*(R_r + R_w*cos(p4))) - dp4*(R_w*cos(phi)*sin(p4)*sin(theta)*sin(xi) - R_w*cos(p4)*cos(theta) + R_w*cos(xi)*sin(p4)*sin(phi)*sin(theta));
   dA(2,7) = 0;
   dA(2,8) = dphi*sin(phi + xi)*cos(theta)*(R_r + R_w*cos(p4)) + dtheta*cos(phi + xi)*sin(theta)*(R_r + R_w*cos(p4)) + dxi*sin(phi + xi)*cos(theta)*(R_r + R_w*cos(p4)) + R_w*dp4*cos(phi + xi)*cos(theta)*sin(p4);
   dA(3,0) = 0;
   dA(3,1) = 0;
   dA(3,2) = 0;
   dA(3,3) = dphi*(xdf*(cos(delta)*cos(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - cos(delta)*sin(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta))) + xbd*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - zbd*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + (R_f*sin(eta) + R_w*cos(p8)*sin(eta))*(cos(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta))) + (R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(cos(delta)*cos(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - cos(delta)*sin(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta))) - R_w*sin(p8)*(cos(lambda)*sin(delta)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - sin(delta)*sin(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)))) + deta*((R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(cos(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - sin(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta))) + (R_f*sin(eta) + R_w*cos(p8)*sin(eta))*(cos(delta)*cos(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + cos(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) + cos(psi)*sin(delta)*cos(theta))) + dtheta*(xdf*(cos(psi)*sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(psi)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(psi)*cos(theta)*sin(lambda)) - (R_f*sin(eta) + R_w*cos(p8)*sin(eta))*(cos(lambda)*cos(phi)*cos(psi)*cos(theta) + cos(psi)*cos(theta)*sin(lambda)*sin(phi)) + (R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(cos(psi)*sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(psi)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(psi)*cos(theta)*sin(lambda)) + R_w*sin(p8)*(cos(delta)*cos(psi)*sin(theta) + cos(lambda)*cos(psi)*sin(delta)*cos(theta)*sin(phi) - cos(phi)*cos(psi)*sin(delta)*cos(theta)*sin(lambda)) + zbd*cos(phi)*cos(psi)*cos(theta) - xbd*cos(psi)*cos(theta)*sin(phi)) + dp8*(R_w*cos(p8)*(cos(lambda)*sin(delta)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + sin(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - cos(delta)*cos(psi)*cos(theta)) - R_w*sin(eta)*sin(p8)*(cos(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - sin(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta))) + R_w*cos(eta)*sin(p8)*(cos(delta)*cos(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + cos(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) + cos(psi)*sin(delta)*cos(theta))) + ddelta*(xdf*(cos(lambda)*sin(delta)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + sin(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - cos(delta)*cos(psi)*cos(theta)) + (R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(cos(lambda)*sin(delta)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + sin(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - cos(delta)*cos(psi)*cos(theta)) + R_w*sin(p8)*(cos(delta)*cos(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + cos(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) + cos(psi)*sin(delta)*cos(theta))) - dpsi*(xdf*(cos(delta)*cos(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + cos(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(delta)*cos(theta)*sin(psi)) + xbd*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + zbd*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - (R_f*sin(eta) + R_w*cos(p8)*sin(eta))*(cos(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta))) + (R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(cos(delta)*cos(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + cos(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(delta)*cos(theta)*sin(psi)) - R_w*sin(p8)*(cos(lambda)*sin(delta)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + sin(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) + cos(delta)*cos(theta)*sin(psi)));
   dA(3,4) = ddelta*(xdf*(cos(lambda)*sin(delta)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(delta)*sin(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta))) + cos(eta)*(R_f + R_w*cos(p8))*(cos(lambda)*sin(delta)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(delta)*sin(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta))) + R_w*sin(p8)*(cos(delta)*cos(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - cos(delta)*sin(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)))) - dtheta*(xdf*(cos(delta)*cos(lambda)*cos(phi)*cos(theta)*sin(psi) + cos(delta)*cos(theta)*sin(lambda)*sin(phi)*sin(psi)) - R_w*sin(p8)*(cos(lambda)*cos(phi)*sin(delta)*cos(theta)*sin(psi) + sin(delta)*cos(theta)*sin(lambda)*sin(phi)*sin(psi)) - sin(eta)*(R_f + R_w*cos(p8))*(cos(lambda)*cos(theta)*sin(phi)*sin(psi) - cos(phi)*cos(theta)*sin(lambda)*sin(psi)) + cos(eta)*(R_f + R_w*cos(p8))*(cos(delta)*cos(lambda)*cos(phi)*cos(theta)*sin(psi) + cos(delta)*cos(theta)*sin(lambda)*sin(phi)*sin(psi)) + xbd*cos(phi)*cos(theta)*sin(psi) + zbd*cos(theta)*sin(phi)*sin(psi)) - dphi*(xdf*(cos(delta)*cos(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + cos(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta))) + xbd*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + zbd*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(eta)*(R_f + R_w*cos(p8))*(cos(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta))) + cos(eta)*(R_f + R_w*cos(p8))*(cos(delta)*cos(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + cos(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta))) - R_w*sin(p8)*(cos(lambda)*sin(delta)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + sin(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)))) + dpsi*(xdf*(cos(delta)*cos(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - cos(delta)*sin(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta))) + xbd*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - zbd*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + sin(eta)*(R_f + R_w*cos(p8))*(cos(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta))) + cos(eta)*(R_f + R_w*cos(p8))*(cos(delta)*cos(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - cos(delta)*sin(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta))) - R_w*sin(p8)*(cos(lambda)*sin(delta)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - sin(delta)*sin(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)))) + dp8*(R_w*cos(p8)*(cos(lambda)*sin(delta)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(delta)*sin(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta))) + R_w*sin(eta)*sin(p8)*(cos(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta))) + R_w*cos(eta)*sin(p8)*(cos(delta)*cos(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - cos(delta)*sin(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)))) - deta*(cos(eta)*(R_f + R_w*cos(p8))*(cos(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta))) - sin(eta)*(R_f + R_w*cos(p8))*(cos(delta)*cos(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - cos(delta)*sin(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta))));
   dA(3,5) = dpsi*((R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(cos(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - sin(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta))) + (R_f*sin(eta) + R_w*cos(p8)*sin(eta))*(cos(delta)*cos(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + cos(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) + cos(psi)*sin(delta)*cos(theta))) + deta*((R_f*sin(eta) + R_w*cos(p8)*sin(eta))*(cos(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta))) - (R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(cos(delta)*cos(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + cos(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(delta)*cos(theta)*sin(psi))) - dphi*((R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(cos(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta))) - (R_f*sin(eta) + R_w*cos(p8)*sin(eta))*(cos(delta)*cos(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - cos(delta)*sin(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)))) - dtheta*((R_f*sin(eta) + R_w*cos(p8)*sin(eta))*(sin(delta)*sin(psi)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi)*sin(psi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)*sin(psi)) + (R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(cos(lambda)*cos(phi)*cos(theta)*sin(psi) + cos(theta)*sin(lambda)*sin(phi)*sin(psi))) + dp8*(R_w*cos(eta)*sin(p8)*(cos(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta))) + R_w*sin(eta)*sin(p8)*(cos(delta)*cos(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + cos(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(delta)*cos(theta)*sin(psi))) + ddelta*(R_f*sin(eta) + R_w*cos(p8)*sin(eta))*(cos(lambda)*sin(delta)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + sin(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) + cos(delta)*cos(theta)*sin(psi));
   dA(3,6) = dpsi*(xdf*cos(psi)*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) + cos(eta)*cos(psi)*(R_f + R_w*cos(p8))*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) + zbd*cos(phi)*cos(psi)*cos(theta) + R_w*cos(psi)*sin(p8)*(cos(delta)*sin(theta) + cos(lambda)*sin(delta)*cos(theta)*sin(phi) - cos(phi)*sin(delta)*cos(theta)*sin(lambda)) - xbd*cos(psi)*cos(theta)*sin(phi) - cos(psi)*cos(theta)*sin(eta)*cos(lambda - phi)*(R_f + R_w*cos(p8))) - dphi*(xdf*sin(psi)*(cos(delta)*cos(lambda)*cos(phi)*cos(theta) + cos(delta)*cos(theta)*sin(lambda)*sin(phi)) + cos(eta)*sin(psi)*(R_f + R_w*cos(p8))*(cos(delta)*cos(lambda)*cos(phi)*cos(theta) + cos(delta)*cos(theta)*sin(lambda)*sin(phi)) - R_w*sin(p8)*sin(psi)*(cos(lambda)*cos(phi)*sin(delta)*cos(theta) + sin(delta)*cos(theta)*sin(lambda)*sin(phi)) + xbd*cos(phi)*cos(theta)*sin(psi) + zbd*cos(theta)*sin(phi)*sin(psi) + sin(lambda - phi)*cos(theta)*sin(eta)*sin(psi)*(R_f + R_w*cos(p8))) + ddelta*(xdf*sin(psi)*(cos(delta)*sin(theta) + cos(lambda)*sin(delta)*cos(theta)*sin(phi) - cos(phi)*sin(delta)*cos(theta)*sin(lambda)) + cos(eta)*sin(psi)*(R_f + R_w*cos(p8))*(cos(delta)*sin(theta) + cos(lambda)*sin(delta)*cos(theta)*sin(phi) - cos(phi)*sin(delta)*cos(theta)*sin(lambda)) - R_w*sin(p8)*sin(psi)*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda))) - deta*(sin(eta)*sin(psi)*(R_f + R_w*cos(p8))*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) + cos(eta)*cos(theta)*sin(psi)*cos(lambda - phi)*(R_f + R_w*cos(p8))) + dtheta*(xdf*sin(psi)*(sin(delta)*cos(theta) + cos(delta)*cos(lambda)*sin(phi)*sin(theta) - cos(delta)*cos(phi)*sin(lambda)*sin(theta)) + cos(eta)*sin(psi)*(R_f + R_w*cos(p8))*(sin(delta)*cos(theta) + cos(delta)*cos(lambda)*sin(phi)*sin(theta) - cos(delta)*cos(phi)*sin(lambda)*sin(theta)) - zbd*cos(phi)*sin(psi)*sin(theta) + R_w*sin(p8)*sin(psi)*(cos(delta)*cos(theta) - cos(lambda)*sin(delta)*sin(phi)*sin(theta) + cos(phi)*sin(delta)*sin(lambda)*sin(theta)) + xbd*sin(phi)*sin(psi)*sin(theta) + sin(eta)*sin(psi)*sin(theta)*cos(lambda - phi)*(R_f + R_w*cos(p8))) + dp8*(R_w*cos(p8)*sin(psi)*(cos(delta)*sin(theta) + cos(lambda)*sin(delta)*cos(theta)*sin(phi) - cos(phi)*sin(delta)*cos(theta)*sin(lambda)) - R_w*cos(eta)*sin(p8)*sin(psi)*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) + R_w*cos(theta)*sin(eta)*sin(p8)*sin(psi)*cos(lambda - phi));
   dA(3,7) = dphi*(xdf*(cos(lambda)*sin(delta)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(delta)*sin(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta))) + cos(eta)*(R_f + R_w*cos(p8))*(cos(lambda)*sin(delta)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(delta)*sin(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta))) + R_w*sin(p8)*(cos(delta)*cos(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - cos(delta)*sin(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)))) - dp8*(R_w*cos(p8)*(cos(delta)*cos(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + cos(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(delta)*cos(theta)*sin(psi)) - R_w*cos(eta)*sin(p8)*(cos(lambda)*sin(delta)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + sin(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) + cos(delta)*cos(theta)*sin(psi))) + dpsi*(xdf*(cos(lambda)*sin(delta)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + sin(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - cos(delta)*cos(psi)*cos(theta)) + cos(eta)*(R_f + R_w*cos(p8))*(cos(lambda)*sin(delta)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + sin(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - cos(delta)*cos(psi)*cos(theta)) + R_w*sin(p8)*(cos(delta)*cos(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + cos(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) + cos(psi)*sin(delta)*cos(theta))) - ddelta*(xdf*(cos(delta)*cos(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + cos(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(delta)*cos(theta)*sin(psi)) + cos(eta)*(R_f + R_w*cos(p8))*(cos(delta)*cos(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + cos(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(delta)*cos(theta)*sin(psi)) - R_w*sin(p8)*(cos(lambda)*sin(delta)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + sin(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) + cos(delta)*cos(theta)*sin(psi))) + dtheta*(xdf*(cos(delta)*sin(psi)*sin(theta) + cos(lambda)*sin(delta)*cos(theta)*sin(phi)*sin(psi) - cos(phi)*sin(delta)*cos(theta)*sin(lambda)*sin(psi)) - R_w*sin(p8)*(sin(delta)*sin(psi)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi)*sin(psi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)*sin(psi)) + cos(eta)*(R_f + R_w*cos(p8))*(cos(delta)*sin(psi)*sin(theta) + cos(lambda)*sin(delta)*cos(theta)*sin(phi)*sin(psi) - cos(phi)*sin(delta)*cos(theta)*sin(lambda)*sin(psi))) + deta*sin(eta)*(R_f + R_w*cos(p8))*(cos(lambda)*sin(delta)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + sin(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) + cos(delta)*cos(theta)*sin(psi));
   dA(3,8) = 0;
   dA(4,0) = 0;
   dA(4,1) = 0;
   dA(4,2) = 0;
   dA(4,3) = dtheta*((R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(sin(delta)*sin(psi)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi)*sin(psi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)*sin(psi)) + xdf*(sin(delta)*sin(psi)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi)*sin(psi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)*sin(psi)) - (R_f*sin(eta) + R_w*cos(p8)*sin(eta))*(cos(lambda)*cos(phi)*cos(theta)*sin(psi) + cos(theta)*sin(lambda)*sin(phi)*sin(psi)) + R_w*sin(p8)*(cos(delta)*sin(psi)*sin(theta) + cos(lambda)*sin(delta)*cos(theta)*sin(phi)*sin(psi) - cos(phi)*sin(delta)*cos(theta)*sin(lambda)*sin(psi)) + zbd*cos(phi)*cos(theta)*sin(psi) - xbd*cos(theta)*sin(phi)*sin(psi)) - deta*((R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(cos(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta))) + (R_f*sin(eta) + R_w*cos(p8)*sin(eta))*(cos(delta)*cos(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + cos(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(delta)*cos(theta)*sin(psi))) - dphi*(xdf*(cos(delta)*cos(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - cos(delta)*sin(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta))) + xbd*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - zbd*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + (R_f*sin(eta) + R_w*cos(p8)*sin(eta))*(cos(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta))) + (R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(cos(delta)*cos(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - cos(delta)*sin(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta))) - R_w*sin(p8)*(cos(lambda)*sin(delta)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(delta)*sin(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)))) - dpsi*(xdf*(cos(delta)*cos(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + cos(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) + cos(psi)*sin(delta)*cos(theta)) + xbd*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + zbd*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - (R_f*sin(eta) + R_w*cos(p8)*sin(eta))*(cos(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - sin(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta))) + (R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(cos(delta)*cos(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + cos(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) + cos(psi)*sin(delta)*cos(theta)) - R_w*sin(p8)*(cos(lambda)*sin(delta)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + sin(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - cos(delta)*cos(psi)*cos(theta))) - dp8*(R_w*cos(p8)*(cos(lambda)*sin(delta)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + sin(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) + cos(delta)*cos(theta)*sin(psi)) - R_w*sin(eta)*sin(p8)*(cos(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta))) + R_w*cos(eta)*sin(p8)*(cos(delta)*cos(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + cos(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(delta)*cos(theta)*sin(psi))) - ddelta*(xdf*(cos(lambda)*sin(delta)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + sin(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) + cos(delta)*cos(theta)*sin(psi)) + (R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(cos(lambda)*sin(delta)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + sin(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) + cos(delta)*cos(theta)*sin(psi)) + R_w*sin(p8)*(cos(delta)*cos(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + cos(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(delta)*cos(theta)*sin(psi)));
   dA(4,4) = ddelta*(xdf*(cos(lambda)*sin(delta)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - sin(delta)*sin(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta))) + (R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(cos(lambda)*sin(delta)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - sin(delta)*sin(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta))) + R_w*sin(p8)*(cos(delta)*cos(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - cos(delta)*sin(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)))) - dpsi*(xdf*(cos(delta)*cos(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - cos(delta)*sin(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta))) + xbd*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - zbd*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + (R_f*sin(eta) + R_w*cos(p8)*sin(eta))*(cos(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta))) + (R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(cos(delta)*cos(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - cos(delta)*sin(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta))) - R_w*sin(p8)*(cos(lambda)*sin(delta)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(delta)*sin(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)))) - dphi*(xdf*(cos(delta)*cos(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + cos(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta))) + xbd*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + zbd*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - (R_f*sin(eta) + R_w*cos(p8)*sin(eta))*(cos(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - sin(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta))) + (R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(cos(delta)*cos(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + cos(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta))) - R_w*sin(p8)*(cos(lambda)*sin(delta)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + sin(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)))) - deta*((R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(cos(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta))) - (R_f*sin(eta) + R_w*cos(p8)*sin(eta))*(cos(delta)*cos(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - cos(delta)*sin(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)))) + dtheta*(xdf*(cos(delta)*cos(lambda)*cos(phi)*cos(psi)*cos(theta) + cos(delta)*cos(psi)*cos(theta)*sin(lambda)*sin(phi)) - (R_f*sin(eta) + R_w*cos(p8)*sin(eta))*(cos(lambda)*cos(psi)*cos(theta)*sin(phi) - cos(phi)*cos(psi)*cos(theta)*sin(lambda)) + (R_f*cos(eta) + R_w*cos(eta)*cos(p8))*(cos(delta)*cos(lambda)*cos(phi)*cos(psi)*cos(theta) + cos(delta)*cos(psi)*cos(theta)*sin(lambda)*sin(phi)) - R_w*sin(p8)*(cos(lambda)*cos(phi)*cos(psi)*sin(delta)*cos(theta) + cos(psi)*sin(delta)*cos(theta)*sin(lambda)*sin(phi)) + xbd*cos(phi)*cos(psi)*cos(theta) + zbd*cos(psi)*cos(theta)*sin(phi)) + dp8*(R_w*cos(p8)*(cos(lambda)*sin(delta)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - sin(delta)*sin(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta))) + R_w*sin(eta)*sin(p8)*(cos(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta))) + R_w*cos(eta)*sin(p8)*(cos(delta)*cos(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - cos(delta)*sin(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta))));
   dA(4,5) = dtheta*(cos(eta)*(R_f + R_w*cos(p8))*(cos(lambda)*cos(phi)*cos(psi)*cos(theta) + cos(psi)*cos(theta)*sin(lambda)*sin(phi)) + sin(eta)*(R_f + R_w*cos(p8))*(cos(psi)*sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(psi)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(psi)*cos(theta)*sin(lambda))) + dp8*(R_w*cos(eta)*sin(p8)*(cos(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - sin(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta))) + R_w*sin(eta)*sin(p8)*(cos(delta)*cos(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + cos(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) + cos(psi)*sin(delta)*cos(theta))) + deta*(sin(eta)*(R_f + R_w*cos(p8))*(cos(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - sin(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta))) - cos(eta)*(R_f + R_w*cos(p8))*(cos(delta)*cos(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + cos(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) + cos(psi)*sin(delta)*cos(theta))) - dpsi*(cos(eta)*(R_f + R_w*cos(p8))*(cos(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta))) + sin(eta)*(R_f + R_w*cos(p8))*(cos(delta)*cos(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + cos(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(delta)*cos(theta)*sin(psi))) - dphi*(cos(eta)*(R_f + R_w*cos(p8))*(cos(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta))) - sin(eta)*(R_f + R_w*cos(p8))*(cos(delta)*cos(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - cos(delta)*sin(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)))) + ddelta*sin(eta)*(R_f + R_w*cos(p8))*(cos(lambda)*sin(delta)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + sin(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - cos(delta)*cos(psi)*cos(theta));
   dA(4,6) = deta*(cos(psi)*sin(eta)*(R_f + R_w*cos(p8))*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) + cos(eta)*cos(psi)*cos(theta)*cos(lambda - phi)*(R_f + R_w*cos(p8))) - ddelta*(xdf*cos(psi)*(cos(delta)*sin(theta) + cos(lambda)*sin(delta)*cos(theta)*sin(phi) - cos(phi)*sin(delta)*cos(theta)*sin(lambda)) + cos(eta)*cos(psi)*(R_f + R_w*cos(p8))*(cos(delta)*sin(theta) + cos(lambda)*sin(delta)*cos(theta)*sin(phi) - cos(phi)*sin(delta)*cos(theta)*sin(lambda)) - R_w*cos(psi)*sin(p8)*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda))) + dpsi*(xdf*sin(psi)*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) + cos(eta)*sin(psi)*(R_f + R_w*cos(p8))*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) + zbd*cos(phi)*cos(theta)*sin(psi) + R_w*sin(p8)*sin(psi)*(cos(delta)*sin(theta) + cos(lambda)*sin(delta)*cos(theta)*sin(phi) - cos(phi)*sin(delta)*cos(theta)*sin(lambda)) - xbd*cos(theta)*sin(phi)*sin(psi) - cos(theta)*sin(eta)*sin(psi)*cos(lambda - phi)*(R_f + R_w*cos(p8))) - dtheta*(xdf*cos(psi)*(sin(delta)*cos(theta) + cos(delta)*cos(lambda)*sin(phi)*sin(theta) - cos(delta)*cos(phi)*sin(lambda)*sin(theta)) + cos(eta)*cos(psi)*(R_f + R_w*cos(p8))*(sin(delta)*cos(theta) + cos(delta)*cos(lambda)*sin(phi)*sin(theta) - cos(delta)*cos(phi)*sin(lambda)*sin(theta)) - zbd*cos(phi)*cos(psi)*sin(theta) + R_w*cos(psi)*sin(p8)*(cos(delta)*cos(theta) - cos(lambda)*sin(delta)*sin(phi)*sin(theta) + cos(phi)*sin(delta)*sin(lambda)*sin(theta)) + xbd*cos(psi)*sin(phi)*sin(theta) + cos(psi)*sin(eta)*sin(theta)*cos(lambda - phi)*(R_f + R_w*cos(p8))) - dp8*(R_w*cos(p8)*cos(psi)*(cos(delta)*sin(theta) + cos(lambda)*sin(delta)*cos(theta)*sin(phi) - cos(phi)*sin(delta)*cos(theta)*sin(lambda)) - R_w*cos(eta)*cos(psi)*sin(p8)*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) + R_w*cos(psi)*cos(theta)*sin(eta)*sin(p8)*cos(lambda - phi)) + dphi*(xdf*cos(psi)*(cos(delta)*cos(lambda)*cos(phi)*cos(theta) + cos(delta)*cos(theta)*sin(lambda)*sin(phi)) - R_w*cos(psi)*sin(p8)*(cos(lambda)*cos(phi)*sin(delta)*cos(theta) + sin(delta)*cos(theta)*sin(lambda)*sin(phi)) + xbd*cos(phi)*cos(psi)*cos(theta) + zbd*cos(psi)*cos(theta)*sin(phi) + cos(eta)*cos(psi)*(R_f + R_w*cos(p8))*(cos(delta)*cos(lambda)*cos(phi)*cos(theta) + cos(delta)*cos(theta)*sin(lambda)*sin(phi)) + sin(lambda - phi)*cos(psi)*cos(theta)*sin(eta)*(R_f + R_w*cos(p8)));
   dA(4,7) = dphi*(xdf*(cos(lambda)*sin(delta)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - sin(delta)*sin(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta))) + cos(eta)*(R_f + R_w*cos(p8))*(cos(lambda)*sin(delta)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - sin(delta)*sin(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta))) + R_w*sin(p8)*(cos(delta)*cos(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - cos(delta)*sin(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)))) - dtheta*(xdf*(cos(delta)*cos(psi)*sin(theta) + cos(lambda)*cos(psi)*sin(delta)*cos(theta)*sin(phi) - cos(phi)*cos(psi)*sin(delta)*cos(theta)*sin(lambda)) + cos(eta)*(R_f + R_w*cos(p8))*(cos(delta)*cos(psi)*sin(theta) + cos(lambda)*cos(psi)*sin(delta)*cos(theta)*sin(phi) - cos(phi)*cos(psi)*sin(delta)*cos(theta)*sin(lambda)) - R_w*sin(p8)*(cos(psi)*sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(psi)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(psi)*cos(theta)*sin(lambda))) - dp8*(R_w*cos(p8)*(cos(delta)*cos(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + cos(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) + cos(psi)*sin(delta)*cos(theta)) - R_w*cos(eta)*sin(p8)*(cos(lambda)*sin(delta)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + sin(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - cos(delta)*cos(psi)*cos(theta))) - ddelta*(xdf*(cos(delta)*cos(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + cos(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) + cos(psi)*sin(delta)*cos(theta)) + cos(eta)*(R_f + R_w*cos(p8))*(cos(delta)*cos(lambda)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + cos(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) + cos(psi)*sin(delta)*cos(theta)) - R_w*sin(p8)*(cos(lambda)*sin(delta)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + sin(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - cos(delta)*cos(psi)*cos(theta))) - dpsi*(xdf*(cos(lambda)*sin(delta)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + sin(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) + cos(delta)*cos(theta)*sin(psi)) + cos(eta)*(R_f + R_w*cos(p8))*(cos(lambda)*sin(delta)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + sin(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) + cos(delta)*cos(theta)*sin(psi)) + R_w*sin(p8)*(cos(delta)*cos(lambda)*(cos(phi)*cos(psi) - sin(phi)*sin(psi)*sin(theta)) + cos(delta)*sin(lambda)*(cos(psi)*sin(phi) + cos(phi)*sin(psi)*sin(theta)) - sin(delta)*cos(theta)*sin(psi))) + deta*sin(eta)*(R_f + R_w*cos(p8))*(cos(lambda)*sin(delta)*(cos(phi)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) + sin(delta)*sin(lambda)*(sin(phi)*sin(psi) - cos(phi)*cos(psi)*sin(theta)) - cos(delta)*cos(psi)*cos(theta));
   dA(4,8) = 0;
   dA(5,0) = 0;
   dA(5,1) = 0;
   dA(5,2) = 0;
   dA(5,3) = 0;
   dA(5,4) = dp8*(R_w*cos(p8)*sin(delta)*cos(theta)*cos(lambda - phi) + R_w*sin(lambda - phi)*cos(theta)*sin(eta)*sin(p8) + R_w*cos(delta)*cos(eta)*cos(theta)*sin(p8)*cos(lambda - phi)) - deta*(sin(lambda - phi)*cos(eta)*cos(theta)*(R_f + R_w*cos(p8)) - cos(delta)*cos(theta)*sin(eta)*cos(lambda - phi)*(R_f + R_w*cos(p8))) - dphi*(zbd*cos(phi)*cos(theta) - xbd*cos(theta)*sin(phi) - cos(theta)*sin(eta)*cos(lambda - phi)*(R_f + R_w*cos(p8)) + xdf*sin(lambda - phi)*cos(delta)*cos(theta) + sin(lambda - phi)*cos(delta)*cos(eta)*cos(theta)*(R_f + R_w*cos(p8)) - R_w*sin(lambda - phi)*sin(delta)*cos(theta)*sin(p8)) + dtheta*(xbd*cos(phi)*sin(theta) + zbd*sin(phi)*sin(theta) + sin(lambda - phi)*sin(eta)*sin(theta)*(R_f + R_w*cos(p8)) + xdf*cos(delta)*sin(theta)*cos(lambda - phi) + cos(delta)*cos(eta)*sin(theta)*cos(lambda - phi)*(R_f + R_w*cos(p8)) - R_w*sin(delta)*sin(p8)*sin(theta)*cos(lambda - phi)) + ddelta*(xdf*sin(delta)*cos(theta)*cos(lambda - phi) + cos(eta)*sin(delta)*cos(theta)*cos(lambda - phi)*(R_f + R_w*cos(p8)) + R_w*cos(delta)*cos(theta)*sin(p8)*cos(lambda - phi));
   dA(5,5) = dphi*(sin(eta)*(R_f + R_w*cos(p8))*(cos(delta)*cos(lambda)*cos(phi)*cos(theta) + cos(delta)*cos(theta)*sin(lambda)*sin(phi)) - sin(lambda - phi)*cos(eta)*cos(theta)*(R_f + R_w*cos(p8))) + dp8*(R_w*sin(eta)*sin(p8)*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) + R_w*cos(eta)*cos(theta)*sin(p8)*cos(lambda - phi)) - deta*(cos(eta)*(R_f + R_w*cos(p8))*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) - cos(theta)*sin(eta)*cos(lambda - phi)*(R_f + R_w*cos(p8))) - dtheta*(sin(eta)*(R_f + R_w*cos(p8))*(sin(delta)*cos(theta) + cos(delta)*cos(lambda)*sin(phi)*sin(theta) - cos(delta)*cos(phi)*sin(lambda)*sin(theta)) - cos(eta)*sin(theta)*cos(lambda - phi)*(R_f + R_w*cos(p8))) - ddelta*sin(eta)*(R_f + R_w*cos(p8))*(cos(delta)*sin(theta) + cos(lambda)*sin(delta)*cos(theta)*sin(phi) - cos(phi)*sin(delta)*cos(theta)*sin(lambda));
   dA(5,6) = dphi*(xdf*(cos(delta)*cos(lambda)*cos(phi)*sin(theta) + cos(delta)*sin(lambda)*sin(phi)*sin(theta)) + cos(eta)*(R_f + R_w*cos(p8))*(cos(delta)*cos(lambda)*cos(phi)*sin(theta) + cos(delta)*sin(lambda)*sin(phi)*sin(theta)) + xbd*cos(phi)*sin(theta) - R_w*sin(p8)*(cos(lambda)*cos(phi)*sin(delta)*sin(theta) + sin(delta)*sin(lambda)*sin(phi)*sin(theta)) + zbd*sin(phi)*sin(theta) + sin(lambda - phi)*sin(eta)*sin(theta)*(R_f + R_w*cos(p8))) - dp8*(R_w*cos(eta)*sin(p8)*(sin(delta)*cos(theta) + cos(delta)*cos(lambda)*sin(phi)*sin(theta) - cos(delta)*cos(phi)*sin(lambda)*sin(theta)) - R_w*cos(p8)*(cos(delta)*cos(theta) - cos(lambda)*sin(delta)*sin(phi)*sin(theta) + cos(phi)*sin(delta)*sin(lambda)*sin(theta)) + R_w*sin(eta)*sin(p8)*sin(theta)*cos(lambda - phi)) - dtheta*(xdf*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) + cos(eta)*(R_f + R_w*cos(p8))*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) + zbd*cos(phi)*cos(theta) + R_w*sin(p8)*(cos(delta)*sin(theta) + cos(lambda)*sin(delta)*cos(theta)*sin(phi) - cos(phi)*sin(delta)*cos(theta)*sin(lambda)) - xbd*cos(theta)*sin(phi) - cos(theta)*sin(eta)*cos(lambda - phi)*(R_f + R_w*cos(p8))) - deta*(sin(eta)*(R_f + R_w*cos(p8))*(sin(delta)*cos(theta) + cos(delta)*cos(lambda)*sin(phi)*sin(theta) - cos(delta)*cos(phi)*sin(lambda)*sin(theta)) - cos(eta)*sin(theta)*cos(lambda - phi)*(R_f + R_w*cos(p8))) + ddelta*(xdf*(cos(delta)*cos(theta) - cos(lambda)*sin(delta)*sin(phi)*sin(theta) + cos(phi)*sin(delta)*sin(lambda)*sin(theta)) + cos(eta)*(R_f + R_w*cos(p8))*(cos(delta)*cos(theta) - cos(lambda)*sin(delta)*sin(phi)*sin(theta) + cos(phi)*sin(delta)*sin(lambda)*sin(theta)) - R_w*sin(p8)*(sin(delta)*cos(theta) + cos(delta)*cos(lambda)*sin(phi)*sin(theta) - cos(delta)*cos(phi)*sin(lambda)*sin(theta)));
   dA(5,7) = dphi*(xdf*(cos(lambda)*cos(phi)*sin(delta)*cos(theta) + sin(delta)*cos(theta)*sin(lambda)*sin(phi)) + cos(eta)*(R_f + R_w*cos(p8))*(cos(lambda)*cos(phi)*sin(delta)*cos(theta) + sin(delta)*cos(theta)*sin(lambda)*sin(phi)) + R_w*sin(p8)*(cos(delta)*cos(lambda)*cos(phi)*cos(theta) + cos(delta)*cos(theta)*sin(lambda)*sin(phi))) - ddelta*(xdf*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) + cos(eta)*(R_f + R_w*cos(p8))*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) + R_w*sin(p8)*(cos(delta)*sin(theta) + cos(lambda)*sin(delta)*cos(theta)*sin(phi) - cos(phi)*sin(delta)*cos(theta)*sin(lambda))) - dp8*(R_w*cos(p8)*(sin(delta)*sin(theta) - cos(delta)*cos(lambda)*cos(theta)*sin(phi) + cos(delta)*cos(phi)*cos(theta)*sin(lambda)) + R_w*cos(eta)*sin(p8)*(cos(delta)*sin(theta) + cos(lambda)*sin(delta)*cos(theta)*sin(phi) - cos(phi)*sin(delta)*cos(theta)*sin(lambda))) + dtheta*(xdf*(cos(delta)*cos(theta) - cos(lambda)*sin(delta)*sin(phi)*sin(theta) + cos(phi)*sin(delta)*sin(lambda)*sin(theta)) + cos(eta)*(R_f + R_w*cos(p8))*(cos(delta)*cos(theta) - cos(lambda)*sin(delta)*sin(phi)*sin(theta) + cos(phi)*sin(delta)*sin(lambda)*sin(theta)) - R_w*sin(p8)*(sin(delta)*cos(theta) + cos(delta)*cos(lambda)*sin(phi)*sin(theta) - cos(delta)*cos(phi)*sin(lambda)*sin(theta))) - deta*sin(eta)*(R_f + R_w*cos(p8))*(cos(delta)*sin(theta) + cos(lambda)*sin(delta)*cos(theta)*sin(phi) - cos(phi)*sin(delta)*cos(theta)*sin(lambda));
   dA(5,8) = 0;
   
   dXd = -A.leftCols<6>().inverse()*dA.rightCols<3>() + A.leftCols<6>().inverse()*dA.leftCols<6>()*A.leftCols<6>().inverse()*A.rightCols<3>();
   m_vel.dX.topRows<4>() = dXd.topRows<4>();
   m_vel.dX.row(5) = dXd.row(4);
   m_vel.dX.row(7) = dXd.row(5);

}

void DynamicSolution::dynamic_matrices(Matrix<double, 5, 1> x, Vector3d v)
{  
   // normalize(md.model, x);
   Matrix<double, 6, 1> pc = for_kin(x.middleRows<2>(3), m_solution_0, ep_0, it_max_0);
   Matrix<double, 8, 1> x_vel;
   x_vel << x(2), x(3), pc(1), x(4), pc(2), pc(3), pc(4), pc(5); 
   Vel_Jacobian(x_vel, v);
   
   Matrix<double, 12, 1> x_pino;
   Matrix<double, 9, 1> dx_pino, dq;

   m_Rb = yrp2rot(x(2), x(3), pc(1));
   m_Bw = yrp2omega(x(2), x(3), pc(1));
   m_dBw = d_yrp2omega(x(2), x(3), pc(1), m_vel.dq_d(3), v(0), m_vel.dq_d(4));
   Vector3d d_yrp;
   d_yrp << m_vel.dq_d(3), v(0), m_vel.dq_d(4);
   Quaterniond quat_b(m_Rb);

   x_pino(0) = x(0); x_pino(1) = x(1); x_pino(2) = pc(0);
   x_pino.middleRows<4>(3) << quat_b.x(), quat_b.y(), quat_b.z(), quat_b.w();
   x_pino(7) = x(4); 
   x_pino.bottomRows<4>() << 1, 0, 1, 0;
   
   dx_pino.topRows<3>() = m_Rb.transpose()*m_vel.dq_d.topRows<3>();
   dx_pino.middleRows<3>(3) = m_Bw*d_yrp;
   dx_pino.bottomRows<3>() << v(1), m_vel.dq_d(5), v(2); 

   dq = m_vel.X*v;

   computeAllTerms(md.model, data, x_pino, dx_pino);
   // computeJointJacobiansTimeVariation(md.model, data, x_pino, dx_pino);
   data.M.triangularView<Eigen::StrictlyLower>() = data.M.transpose().triangularView<Eigen::StrictlyLower>();
   
   m_Jqv.block<3,3>(0,0) = m_Rb.transpose();
   m_Jqv.block<3,3>(3,3) = m_Bw;

   m_dJqv.block<3,3>(0,0) = -hat(dx_pino.middleRows<3>(3))*m_Rb.transpose();
   m_dJqv.block<3,3>(3,3) = m_dBw;

   m_dm.M9 = m_Jqv.transpose()*data.M*m_Jqv;
   m_dm.C9 = m_Jqv.transpose()*(data.M*m_dJqv*dq+data.nle);
   
   m_dm.M = m_vel.X.transpose()*m_dm.M9*m_vel.X;
   m_dm.C = m_vel.X.transpose()*(m_dm.M9*m_vel.dX*v+m_dm.C9);

   m_tj.X_f << x(3), x(0), x(1), pc(0), pc(1), x(4);
   m_tj.JX(0,0) = 1;
   m_tj.JX.middleRows<3>(1) = m_vel.X.topRows<3>();
   m_tj.JX.bottomRows<2>() = m_vel.X.middleRows<2>(5);
   m_tj.dJX.middleRows<3>(1) = m_vel.dX.topRows<3>();
   m_tj.dJX.bottomRows<2>() = m_vel.dX.middleRows<2>(5);

}
