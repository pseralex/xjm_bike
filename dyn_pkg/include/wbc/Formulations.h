#ifndef Formulations_H
#define Formulations_H
#pragma once
#include"para/BikeParameters.hpp"
#include "fed/RobotFeed.h"
#include "ref/RefTrajes.h"
#include <qpmad/solver.h>
#include "wbc/tasks/ConstDynamicConsistence.h"
#include "wbc/tasks/ConstJointsTauLimit.h"
#include "wbc/tasks/TaskAll.h"

namespace wbc_controller_xjm
{
   
    class Formulations
    {
    public:
        Formulations(const BikeParameters& conf, const RefTrajes& refTrajes);
        ~Formulations();

        /// ==================================================================== ///    
        void run(const BikeParameters& conf, const RobotFeed& robotFeed, const RefTrajes& refTrajes, int count=0);
        void compute_all_tasks_data(const BikeParameters& conf, const RobotFeed& robotFeed, const RefTrajes& refTrajes);
        void formulation(const BikeParameters& conf, const RobotFeed& robotFeed, const RefTrajes& refTrajes);

        std::vector<std::unique_ptr<Constraints_Base>> m_tasksPtr;
        ConstDynamicConsistence m_constDynamicConsistence; 
        ConstJointsTauLimit m_constJointsTauLimit;
        TaskAll m_taskAll;
    };
} 
#endif