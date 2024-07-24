#include "wbc/constraints/Constraint_Target.h"

namespace wbc_controller_xjm
{
    /// 构造函数
    Constraint_Target::Constraint_Target(const unsigned int rows, const unsigned int block_cols_start, const unsigned int block_cols_num)
    :Constraints_Base(rows, block_cols_start, block_cols_num)
    ,m_b(Vector_h::Zero(rows))
    {
        m_is_target = true;
    }

    /// 析构函数
    Constraint_Target::~Constraint_Target(){}

    const Vector_h & Constraint_Target::vector() const{return m_b;}
    const Vector_h & Constraint_Target::lowerBound() const{assert(false); return m_b;}
    const Vector_h & Constraint_Target::upperBound() const{assert(false); return m_b;}

    Vector_h & Constraint_Target::vector(){return m_b;}
    Vector_h & Constraint_Target::lowerBound(){assert(false); return m_b;}
    Vector_h & Constraint_Target::upperBound(){assert(false); return m_b;}

    bool Constraint_Target::setVector(ConstRefVector_h b){m_b = b; return true;}
    bool Constraint_Target::setLowerBound(ConstRefVector_h lb){assert(false); return false;}
    bool Constraint_Target::setUpperBound(ConstRefVector_h ub){assert(false); return false;}

    bool Constraint_Target::move_to_target()  {
        
        m_is_target   = true;    
        m_is_equality = false;  
        return true;
    };

    bool Constraint_Target::move_to_equality(){

        m_is_equality = true;  
        m_is_target   = false;    
        return true;
    };

}


