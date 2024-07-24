#ifndef TaskAll_H
#define TaskAll_H
#pragma once

#include "wbc/tasks/BaseTasksConst.h"
#include "wbc/constraints/Constraint_Target.h"
#include "wbc/trajes/TrajEuc.hpp"

namespace wbc_controller_xjm
{
    class TaskAll : public BaseTasksConst
    {
    public:
        TaskAll(const BikeParameters& conf, const std::string & name="taskAll");
        ~TaskAll();

        void setRefrence(const RefTrajes& refTrajes);
        void compute(const BikeParameters& conf, const RobotFeed& robotFeed);

        Constraint_Target & getConstraint_tra();

        Constraint_Target m_tra_ref_constraint;
    
    // private:
        TrajEuc m_tra_ref;
    };
} 
#endif
