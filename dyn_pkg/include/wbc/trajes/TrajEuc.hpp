#ifndef TrajEuc_H
#define TrajEuc_H
#pragma once
#include "wbc/fwd/fwd.hpp"

namespace wbc_controller_xjm
{
    class TrajEuc
    {
    public:
        TrajEuc(unsigned int size=6);
        ~TrajEuc();

        void resize(unsigned int size);

        const Vector_h & getPos() const; 
        const Vector_h & getVel() const;
        const Vector_h & getAcc() const;

        void setPos(const Vector_h & pos);
        void setVel(const Vector_h & vel);
        void setAcc(const Vector_h & acc);
        
        void setTrajEuc(const TrajEuc & trajEuc);

        Vector_h m_pos;
        Vector_h m_vel;
        Vector_h m_acc;

    private:

    };
} 
#endif