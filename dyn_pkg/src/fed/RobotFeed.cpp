#include "fed/RobotFeed.h"

namespace wbc_controller_xjm
{
    RobotFeed::RobotFeed(const BikeParameters& conf, const RosInterface& rosInterface)
    {
        x_feedback.setZero(5); 
        v_feedback.setZero(3);
        run(conf, rosInterface);
    }
    RobotFeed::~RobotFeed(){}

    /// 
    void RobotFeed::run(const BikeParameters& conf, const RosInterface& rosInterface){

        Vector3_h yrp = m_dyn_sol.quat2yrp(rosInterface.m_q_base_feedback(3), rosInterface.m_q_base_feedback(4), rosInterface.m_q_base_feedback(5), rosInterface.m_q_base_feedback(6));
                        
        x_feedback << rosInterface.m_q_base_feedback(0), rosInterface.m_q_base_feedback(1), yrp(0), yrp(1), rosInterface.m_q_joints_feedback(0);
        
        Vector3_h omega = rosInterface.m_v_base_feedback.bottomRows<3>();
        Vector3_h dyrp = m_dyn_sol.yrp2omega(yrp(0), yrp(1), yrp(2)).inverse() * omega;

        v_feedback << dyrp(1), rosInterface.m_v_joints_feedback(0), rosInterface.m_v_joints_feedback(2);

        /// 更新动力学库
        m_dyn_sol.dynamic_matrices(x_feedback, v_feedback);                        
       
    }

}
