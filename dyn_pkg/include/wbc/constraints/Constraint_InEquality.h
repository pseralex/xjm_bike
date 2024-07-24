#ifndef Constraint_InEquality_H
#define Constraint_InEquality_H
#pragma once
#include "wbc/constraints/Constraints_Base.h"

namespace wbc_controller_xjm
{
    class Constraint_InEquality : public Constraints_Base
    {
    public:
        Constraint_InEquality(const unsigned int rows, const unsigned int block_cols_start, const unsigned int block_cols_num);
        ~Constraint_InEquality();

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
        Vector_h m_Aub;
        Vector_h m_Alb;

    };
} 
#endif