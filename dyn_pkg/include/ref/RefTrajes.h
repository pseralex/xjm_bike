#ifndef RefTrajes_H
#define RefTrajes_H
#pragma once
#include "para/BikeParameters.hpp"
#include "fed/RobotFeed.h"
#include "wbc/dataRW/DataRW.h"

namespace wbc_controller_xjm
{
    class RefTrajes
    {
    public:
        RefTrajes(const BikeParameters& conf);
        ~RefTrajes();

        void set_initial_value(const BikeParameters& conf);

        void read_motion_data_fr_txt(const BikeParameters& conf);

        void perform_motion(const BikeParameters& conf, const int & count=0);
        
        Vector_h m_tra_ref, m_dtra_ref, m_ddtra_ref;
        Matrix_h m_tra_ref_matrix, m_dtra_ref_matrix, m_ddtra_ref_matrix; 
        DataRW m_dataRW;

    private:
    }; 
} 
#endif