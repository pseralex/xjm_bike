#include "wbc/qpSolve/QpWbc.h"

namespace wbc_controller_xjm
{
    QpWbc::QpWbc(const BikeParameters& conf, const RefTrajes& refTrajes)
    :m_formulations(conf, refTrajes)
    ,m_x(Vector_h::Zero(conf.m_x_dim))
    ,m_q_all_ddot(Vector_h::Zero(conf.m_nvi))
    ,m_tau_qp(Vector_h::Zero(conf.m_ntau))
    {
        reInitialize_H_g(conf);
        reInitialize_A_b(conf);  
    }
    QpWbc::~QpWbc(){}

    /// 主函数
    void QpWbc::run(const BikeParameters& conf, const RobotFeed& robotFeed, const RefTrajes& refTrajes, int count){

        /// ==========================================基于qp的wbc以及ID控制器========================================== ///
        m_formulations.run(conf, robotFeed, refTrajes, count);
        qp_id_controller(conf, robotFeed, refTrajes);  
        /// ==========================================基于qp的wbc以及ID控制器========================================== ///
    }

    ///
    void QpWbc::formulation(const BikeParameters& conf, const RobotFeed& robotFeed, const RefTrajes& refTrajes){

        /// 构造QP数据
        reInitialize_H_g(conf);
        reInitialize_A_b(conf);
        int row_index = 0;
        for (const auto& it : m_formulations.m_tasksPtr){

            if(it->isTarget()&&it->isActivated()){

                m_H.noalias() += 0.5*it->matrix().transpose()*it->matrix();
                m_g.noalias() -= it->matrix().transpose()*it->vector();
                m_targets_num += 1;
            }
            else if(it->isEquality()&&it->isActivated()){

                m_A.middleRows(row_index, it->rows()) = it->matrix();
                m_Au.segment(row_index,   it->rows()) = it->vector();
                m_Al.segment(row_index,   it->rows()) = it->vector();
                row_index += it->rows();
            }
            else if(it->isInequality()&&it->isActivated()){

                m_A.middleRows(row_index, it->rows()) = it->matrix();
                m_Au.segment(row_index,   it->rows()) = it->upperBound();
                m_Al.segment(row_index,   it->rows()) = it->lowerBound();
                row_index += it->rows();
            }
            else if(it->isZero()&&it->isActivated()){

                m_A.middleRows(row_index, it->rows()) = it->matrix();
                m_Au.segment(row_index,   it->rows()) = it->vector();
                m_Al.segment(row_index,   it->rows()) = it->vector();
                row_index += it->rows();
            }
            else if(it->isBound()&&it->isActivated()){
                /// 边界处理，目前暂时无边界
            }
        }
        m_H += conf.m_decision_variable_amp_limit;
    }

    /// qp求解和基于ID的控制
    void QpWbc::qp_id_controller(const BikeParameters& conf, const RobotFeed& robotFeed, const RefTrajes& refTrajes){

        /// 构造QP矩阵向量
        formulation(conf, robotFeed, refTrajes);

        /// QP求解
        m_solve_status = m_solver.solve(m_x, m_H, m_g, Eigen::VectorXd(), Eigen::VectorXd(), m_A, m_Al, m_Au);

        /// 
        m_q_all_ddot = m_x.head(conf.m_nvi);
        m_tau_qp = m_x.tail(conf.m_ntau);
        
    }

    /// 计算约束的个数
    int QpWbc::get_constraints_num(){
        int c_num = 0;
        for(const auto& it : m_formulations.m_tasksPtr){if(it->isEquality()||it->isInequality()||it->isZero()){c_num += it->rows();}}
        return c_num;
    }

    /// 重新初始化约束矩阵和向量
    void QpWbc::reInitialize_A_b(const BikeParameters& conf){
        m_constraints_num = get_constraints_num();
        m_A  = Matrix_h::Zero(m_constraints_num, conf.m_x_dim);   // constraint matrix
        m_Al = Vector_h::Zero(m_constraints_num);                 // constraints lower bound
        m_Au = Vector_h::Zero(m_constraints_num);                 // constraints upper bound
    }

    /// 重新初始化目标矩阵和向量
    void QpWbc::reInitialize_H_g(const BikeParameters& conf){
        m_targets_num = 0;
        m_H = Matrix_h::Zero(conf.m_x_dim, conf.m_x_dim);   // hessian matrix
        m_g = Vector_h::Zero(conf.m_x_dim);                 // gradient vector
    }

    Vector_h& QpWbc::get_tau_qp(){return m_tau_qp;}
} 