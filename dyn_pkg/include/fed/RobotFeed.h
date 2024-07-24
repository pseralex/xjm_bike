#ifndef RobotFeed_H
#define RobotFeed_H
#pragma once
#include "fed/dynamic_solution.hpp"
#include "para/BikeParameters.hpp"
#include "ross/RosInterface.h"

namespace wbc_controller_xjm
{
    class RobotFeed
    {
    public:
        RobotFeed(const BikeParameters& conf, const RosInterface& rosInterface);
        ~RobotFeed();

        void run(const BikeParameters& conf, const RosInterface& rosInterface);

        DynamicSolution m_dyn_sol;
        Vector_h x_feedback;
        Vector_h v_feedback;
        
    private:
    };
} 
#endif