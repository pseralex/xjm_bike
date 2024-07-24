#include "wbc/constraints/Constraint_InEquality.h"

namespace wbc_controller_xjm
{
    /// 构造函数
    Constraint_InEquality::Constraint_InEquality(const unsigned int rows, const unsigned int block_cols_start, const unsigned int block_cols_num)
    :Constraints_Base(rows, block_cols_start, block_cols_num)
    ,m_Aub(Vector_h::Zero(rows))
    ,m_Alb(Vector_h::Zero(rows))
    {
        m_is_inEquality = true;
    }

    /// 析构函数
    Constraint_InEquality::~Constraint_InEquality(){}

    const Vector_h & Constraint_InEquality::vector() const{assert(false); return m_Alb;}
    const Vector_h & Constraint_InEquality::lowerBound() const{ return m_Alb;}
    const Vector_h & Constraint_InEquality::upperBound() const{ return m_Aub;}

    Vector_h & Constraint_InEquality::vector(){assert(false); return m_Alb;}
    Vector_h & Constraint_InEquality::lowerBound(){ return m_Alb;}
    Vector_h & Constraint_InEquality::upperBound(){ return m_Aub;}

    bool Constraint_InEquality::setVector(ConstRefVector_h ){assert(false);    return false;}
    bool Constraint_InEquality::setLowerBound(ConstRefVector_h Alb){m_Alb = Alb; return true;}
    bool Constraint_InEquality::setUpperBound(ConstRefVector_h Aub){m_Aub = Aub; return true;}
}

