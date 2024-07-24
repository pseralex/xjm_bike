#include "wbc/tasks/ConstJointsTauLimit.h"

namespace wbc_controller_xjm
{
    ConstJointsTauLimit::ConstJointsTauLimit(const BikeParameters& conf, const std::string & name)
    :BaseTasksConst(name)
    ,m_ddq_tauLimit(conf.m_x_dim,  0,  conf.m_x_dim)
    {
    }
    ConstJointsTauLimit::~ConstJointsTauLimit(){}

    void ConstJointsTauLimit::compute(const BikeParameters& conf, const RobotFeed& robotFeed){

        Matrix_h Mat = Matrix_h::Identity(5,5);
        m_ddq_tauLimit.setBlock(Mat);
        m_ddq_tauLimit.setUpperBound(conf.m_ddq_tau_upper);
        m_ddq_tauLimit.setLowerBound(conf.m_ddq_tau_lower);

    }

    Constraint_InEquality & ConstJointsTauLimit::getConstraint_ddq_tau(){ return m_ddq_tauLimit; }
}
