#ifndef BaseTasksConst_H
#define BaseTasksConst_H
#pragma once
#include "para/BikeParameters.hpp"
#include "fed/RobotFeed.h"
#include "wbc/constraints/Constraints_Base.h"
#include "ref/RefTrajes.h"

namespace wbc_controller_xjm
{
    class BaseTasksConst
    {
    public:
        BaseTasksConst(const std::string & name="None");
        ~BaseTasksConst();

        const std::string & name() const;
        
        void setName(const std::string & name);

    private:
        std::string m_name;
        
    };
} 
#endif