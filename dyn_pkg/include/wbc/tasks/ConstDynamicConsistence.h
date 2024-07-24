#ifndef ConstDynamicConsistence_H
#define ConstDynamicConsistence_H
#pragma once
#include "wbc/tasks/BaseTasksConst.h"
#include "wbc/constraints/Constraint_Equality.h"

namespace wbc_controller_xjm
{
    class ConstDynamicConsistence : public BaseTasksConst
    {
    public:
        ConstDynamicConsistence(const BikeParameters& conf, const std::string & name="constDynamicConsistence");
        ~ConstDynamicConsistence();

        void compute(const BikeParameters& conf, const RobotFeed& robotFeed);

        Constraint_Equality & getConstraint();
        
    // private:
        Constraint_Equality m_dyn_constraint;
    
    };
} 
#endif