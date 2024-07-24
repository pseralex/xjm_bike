#include "wbc/constraints/Constraint_Equality.h"

namespace wbc_controller_xjm
{
    /// 构造函数
    Constraint_Equality::Constraint_Equality(const unsigned int rows, const unsigned int block_cols_start, const unsigned int block_cols_num)
    :Constraints_Base(rows, block_cols_start, block_cols_num)
    ,m_b(Vector_h::Zero(rows))
    {
        m_is_equality = true;
    }

    /// 析构函数
    Constraint_Equality::~Constraint_Equality(){}

    const Vector_h & Constraint_Equality::vector() const{return m_b;}
    const Vector_h & Constraint_Equality::lowerBound() const{assert(false); return m_b;}
    const Vector_h & Constraint_Equality::upperBound() const{assert(false); return m_b;}

    Vector_h & Constraint_Equality::vector(){return m_b;}
    Vector_h & Constraint_Equality::lowerBound(){assert(false); return m_b;}
    Vector_h & Constraint_Equality::upperBound(){assert(false); return m_b;}

    bool Constraint_Equality::setVector(ConstRefVector_h b){m_b = b; return true;}
    bool Constraint_Equality::setLowerBound(ConstRefVector_h Alb){assert(false); return false;}
    bool Constraint_Equality::setUpperBound(ConstRefVector_h Aub){assert(false); return false;}
}