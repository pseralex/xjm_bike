#ifndef ConstJointsTauLimit_H
#define ConstJointsTauLimit_H
#pragma once

#include "wbc/tasks/BaseTasksConst.h"
#include "wbc/constraints/Constraint_InEquality.h"

namespace wbc_controller_xjm
{
    class ConstJointsTauLimit : public BaseTasksConst
    {
    public:
        ConstJointsTauLimit(const BikeParameters& conf, const std::string & name="constJointsTauLimit");
        ~ConstJointsTauLimit();

        void compute(const BikeParameters& conf, const RobotFeed& robotFeed);

        Constraint_InEquality & getConstraint_ddq_tau();

    // private:
        Constraint_InEquality m_ddq_tauLimit;

    };
} 
#endif