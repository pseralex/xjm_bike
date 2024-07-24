#ifndef Constraint_Bound_H
#define Constraint_Bound_H
#pragma once
#include "wbc/constraints/Constraints_Base.h"

namespace wbc_controller_xjm
{
    class Constraint_Bound : public Constraints_Base
    {
    public:
        Constraint_Bound(const unsigned int nVars=55);
        ~Constraint_Bound();

        const Vector_h & vector() const;
        const Vector_h & lowerBound() const;
        const Vector_h & upperBound() const;

        Vector_h & vector();
        Vector_h & lowerBound();
        Vector_h & upperBound();

        bool setVector(ConstRefVector_h b);
        bool setLowerBound(ConstRefVector_h lb);
        bool setUpperBound(ConstRefVector_h ub);

    private:
        Vector_h m_ub;
        Vector_h m_lb;

    };
} 
#endif