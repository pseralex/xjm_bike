#ifndef Constraint_Target_H
#define Constraint_Target_H
#pragma once
#include "wbc/constraints/Constraints_Base.h"

namespace wbc_controller_xjm
{
    class Constraint_Target : public Constraints_Base
    {
    public:
        Constraint_Target(const unsigned int rows, const unsigned int block_cols_start, const unsigned int block_cols_num);
        ~Constraint_Target();

        const Vector_h & vector() const;
        const Vector_h & lowerBound() const;
        const Vector_h & upperBound() const;

        Vector_h & vector();
        Vector_h & lowerBound();
        Vector_h & upperBound();

        bool setVector(ConstRefVector_h b);
        bool setLowerBound(ConstRefVector_h lb);
        bool setUpperBound(ConstRefVector_h ub);

        bool move_to_target();
        bool move_to_equality();

    private:
        Vector_h m_b;


    };
} 
#endif