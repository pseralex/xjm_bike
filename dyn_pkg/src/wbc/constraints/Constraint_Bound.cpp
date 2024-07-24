#include "wbc/constraints/Constraint_Bound.h"

namespace wbc_controller_xjm
{
    /// 构造函数
    Constraint_Bound::Constraint_Bound(const unsigned int nVars)
    :Constraints_Base(nVars, 0, nVars)
    {
        m_is_bound = true;
        m_A = Matrix_h::Identity(nVars, nVars); 
    }

    /// 析构函数
    Constraint_Bound::~Constraint_Bound(){}

    const Vector_h & Constraint_Bound::vector() const{assert(false); return m_lb;}
    const Vector_h & Constraint_Bound::lowerBound() const{return m_lb;}
    const Vector_h & Constraint_Bound::upperBound() const{return m_ub;}

    Vector_h & Constraint_Bound::vector(){assert(false); return m_lb;}
    Vector_h & Constraint_Bound::lowerBound(){return m_lb;}
    Vector_h & Constraint_Bound::upperBound(){return m_ub;}

    bool Constraint_Bound::setVector(ConstRefVector_h ){assert(false);   return false;}
    bool Constraint_Bound::setLowerBound(ConstRefVector_h lb){m_lb = lb; return true;}
    bool Constraint_Bound::setUpperBound(ConstRefVector_h ub){m_ub = ub; return true;}
}

