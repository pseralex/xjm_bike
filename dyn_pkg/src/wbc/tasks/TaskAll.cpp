#include "wbc/tasks/TaskAll.h"

namespace wbc_controller_xjm
{
    TaskAll::TaskAll(const BikeParameters& conf, const std::string & name)
    :BaseTasksConst(name)
    ,m_tra_ref_constraint(conf.m_ntask, 0, conf.m_nvi)
    ,m_tra_ref(conf.m_ntask)
    {}
    TaskAll::~TaskAll(){}

    void TaskAll::setRefrence(const RefTrajes& refTrajes){

        m_tra_ref.setPos(refTrajes.m_tra_ref);
        m_tra_ref.setVel(refTrajes.m_dtra_ref);
        m_tra_ref.setAcc(refTrajes.m_ddtra_ref);

    }    

    void TaskAll::compute(const BikeParameters& conf, const RobotFeed& robotFeed){

        Vector_h kp = Vector_h::Zero(conf.m_ntask);
        Vector_h kd = Vector_h::Zero(conf.m_ntask);
        Matrix_h weight = Matrix_h::Zero(conf.m_ntask,conf.m_ntask);

        kp = conf.m_kp;
        kd = conf.m_kd;
        weight = conf.m_weight;
        
        Vector_h tra_error = m_tra_ref.getPos() - robotFeed.m_dyn_sol.m_tj.X_f;
        Vector_h dtra_error = m_tra_ref.getVel() - robotFeed.m_dyn_sol.m_tj.JX*robotFeed.v_feedback;
        Vector_h ddtra_des = m_tra_ref.getAcc() + kp.cwiseProduct(tra_error) + kd.cwiseProduct(dtra_error);

        m_tra_ref_constraint.setBlock(weight*robotFeed.m_dyn_sol.m_tj.JX);
        m_tra_ref_constraint.setVector(weight*(ddtra_des-robotFeed.m_dyn_sol.m_tj.dJX*robotFeed.v_feedback));

    }

    Constraint_Target & TaskAll::getConstraint_tra(){ return m_tra_ref_constraint; }
}
