#include "wbc/constraints/Constraint_Zero.h"

namespace wbc_controller_xjm
{
    /// 构造函数
    Constraint_Zero::Constraint_Zero(const unsigned int rows, const unsigned int block_cols_start)
    :Constraints_Base(rows, block_cols_start, rows)
    ,m_b_zero(Vector_h::Zero(rows))
    {
        m_is_zero = true; 
        m_is_activated = false; /// 置零约束默认是未激活的

        m_A.middleCols(block_cols_start, rows) = Matrix_h::Identity(rows, rows); 
    }

    /// 析构函数
    Constraint_Zero::~Constraint_Zero(){}

    const Vector_h & Constraint_Zero::vector() const{return m_b_zero;}
    const Vector_h & Constraint_Zero::lowerBound() const{assert(false); return m_b_zero;}
    const Vector_h & Constraint_Zero::upperBound() const{assert(false); return m_b_zero;}

    Vector_h & Constraint_Zero::vector(){return m_b_zero;}
    Vector_h & Constraint_Zero::lowerBound(){assert(false); return m_b_zero;}
    Vector_h & Constraint_Zero::upperBound(){assert(false); return m_b_zero;}

    bool Constraint_Zero::setVector(ConstRefVector_h b){m_b_zero = Vector_h::Zero(m_b_zero.rows()); return true;}
    bool Constraint_Zero::setLowerBound(ConstRefVector_h Alb){assert(false); return false;}
    bool Constraint_Zero::setUpperBound(ConstRefVector_h Aub){assert(false); return false;}
}


