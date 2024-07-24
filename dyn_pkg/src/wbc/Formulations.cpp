#include "wbc/Formulations.h"

namespace wbc_controller_xjm
{
    Formulations::Formulations(const BikeParameters& conf, const RefTrajes& refTrajes)
    :m_constDynamicConsistence(conf)
    ,m_constJointsTauLimit(conf)
    ,m_taskAll(conf)
    {
    }
    Formulations::~Formulations(){}

    /// 主函数
    void Formulations::run(const BikeParameters& conf, const RobotFeed& robotFeed, const RefTrajes& refTrajes, int count){

        /// ==========================================计算获得QP数据========================================== ///
        compute_all_tasks_data(conf, robotFeed, refTrajes);
        /// ==========================================计算获得QP数据========================================== ///
    }

    /// 计算所有任务的数据
    void Formulations::compute_all_tasks_data(const BikeParameters& conf, const RobotFeed& robotFeed, const RefTrajes& refTrajes){
        /// ========================================== 初始化 ========================================== /// 
        m_tasksPtr.clear(); /// 注意，必须初始化
        
        /// ========================================== 目标 ========================================== /// 
        /// 车身平衡、质心轨迹  
        m_taskAll.setRefrence(refTrajes);      
        m_taskAll.compute(conf, robotFeed);
        m_tasksPtr.push_back(std::make_unique<Constraint_Target>(m_taskAll.getConstraint_tra()));
    
        /// ========================================== 等式约束 ========================================== ///                               
        /// 浮动基动力学一致性
        m_constDynamicConsistence.compute(conf, robotFeed);  
        m_tasksPtr.push_back(std::make_unique<Constraint_Equality>(m_constDynamicConsistence.getConstraint()));

        /// ========================================== 不等式约束 ========================================== ///                               
        /// 独立广义加速度及关节力矩上下限约束
        m_constJointsTauLimit.compute(conf, robotFeed);
        m_tasksPtr.push_back(std::make_unique<Constraint_InEquality>(m_constJointsTauLimit.getConstraint_ddq_tau()));    
    }
} 