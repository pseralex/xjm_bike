#ifndef QpWbc_H
#define QpWbc_H
#pragma once
#include "wbc/Formulations.h"

namespace wbc_controller_xjm
{
    class QpWbc
    {
    public:
        QpWbc(const BikeParameters& conf, const RefTrajes& refTrajes);
        ~QpWbc();

        /// ==================================================================== ///    
        void run(const BikeParameters& conf, const RobotFeed& robotFeed, const RefTrajes& refTrajes, int count=0);
        void formulation(const BikeParameters& conf, const RobotFeed& robotFeed, const RefTrajes& refTrajes);
        void qp_id_controller(const BikeParameters& conf, const RobotFeed& robotFeed, const RefTrajes& refTrajes);
        int  get_constraints_num();
        void reInitialize_H_g(const BikeParameters& conf);
        void reInitialize_A_b(const BikeParameters& conf);
        Vector_h& get_tau_qp();

        /// ==================================================================== ///    
        Formulations m_formulations;

        /// ==================================================================== ///    
        Matrix_h m_H;   // hessian matrix
        Vector_h m_g;   // gradient vector
        Matrix_h m_A;   // constraint matrix
        Vector_h m_Al;  // constraints lower bound
        Vector_h m_Au;  // constraints upper bound
        int m_constraints_num;   /// number of equality-inequality constraints
        int m_targets_num;   /// number of equality-inequality constraints
        Vector_h                    m_x;
        Vector_h                    m_q_all_ddot;
        Vector_h                    m_tau_qp;
        qpmad::Solver               m_solver;
        qpmad::Solver::ReturnStatus m_solve_status;   
        /// ==================================================================== ///            
    };
} 
#endif