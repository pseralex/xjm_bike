#include "wbc/tasks/ConstDynamicConsistence.h"

namespace wbc_controller_xjm
{
    ConstDynamicConsistence::ConstDynamicConsistence(const BikeParameters& conf, const std::string & name)
    :BaseTasksConst(name)
    ,m_dyn_constraint(3, 0, conf.m_x_dim)
    {
    }
    ConstDynamicConsistence::~ConstDynamicConsistence(){}

    void ConstDynamicConsistence::compute(const BikeParameters& conf, const RobotFeed& robotFeed){

        Matrix_h dyn_Mat = Matrix_h::Zero(3, conf.m_x_dim);
        dyn_Mat.leftCols<3>() = robotFeed.m_dyn_sol.m_dm.M;
        dyn_Mat.rightCols<2>() = -robotFeed.m_dyn_sol.m_dm.S;
        
        m_dyn_constraint.setBlock(dyn_Mat);
        m_dyn_constraint.setVector(-robotFeed.m_dyn_sol.m_dm.C);
    }

    Constraint_Equality & ConstDynamicConsistence::getConstraint(){ return m_dyn_constraint;}
}
