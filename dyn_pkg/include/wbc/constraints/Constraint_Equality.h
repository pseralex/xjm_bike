#ifndef Constraint_Equality_H
#define Constraint_Equality_H
#pragma once
#include "wbc/constraints/Constraints_Base.h"

namespace wbc_controller_xjm
{
    class Constraint_Equality : public Constraints_Base
    {
    public:
        Constraint_Equality(const unsigned int rows, const unsigned int block_cols_start, const unsigned int block_cols_num);
        ~Constraint_Equality();

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
        Vector_h m_b;

    };
} 
#endif