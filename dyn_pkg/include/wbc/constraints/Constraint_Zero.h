#ifndef Constraint_Zero_H
#define Constraint_Zero_H
#pragma once
#include "wbc/constraints/Constraints_Base.h"

namespace wbc_controller_xjm
{
    class Constraint_Zero : public Constraints_Base
    {
    public:
        Constraint_Zero(const unsigned int rows, const unsigned int block_cols_start);
        ~Constraint_Zero();

        const Vector_h & vector() const;
        const Vector_h & lowerBound() const;
        const Vector_h & upperBound() const;

        Vector_h & vector();
        Vector_h & lowerBound();
        Vector_h & upperBound();

        bool setVector(ConstRefVector_h b);
        bool setLowerBound(ConstRefVector_h Alb);
        bool setUpperBound(ConstRefVector_h Aub);

    private:
        Vector_h m_b_zero;

    };
} 
#endif