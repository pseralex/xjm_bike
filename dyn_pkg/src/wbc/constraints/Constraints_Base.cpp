#include "wbc/constraints/Constraints_Base.h"

namespace wbc_controller_xjm
{
    /// 构造函数
    Constraints_Base::Constraints_Base(const unsigned int rows, const unsigned int block_cols_start, const unsigned int block_cols_num,  const unsigned int cols)
    :m_block_cols_start(block_cols_start)
    ,m_block_cols_num(block_cols_num)
    ,m_A(Matrix_h::Zero(rows, cols))
    {
        initialize();
    }

    /// 
    void Constraints_Base::initialize(){
        m_is_target      = false;
        m_is_equality    = false;
        m_is_inEquality  = false;
        m_is_bound       = false;  
        m_is_zero        = false;
        m_is_activated   = true;   /// 默认都是激活的
    };

    bool Constraints_Base::isTarget()     const{return m_is_target;};
    bool Constraints_Base::isEquality()   const{return m_is_equality;};
    bool Constraints_Base::isInequality() const{return m_is_inEquality;};
    bool Constraints_Base::isBound()      const{return m_is_bound;};
    bool Constraints_Base::isZero()       const{return m_is_zero;};

    void Constraints_Base::setActivated(bool is_activated){m_is_activated = is_activated;}; 
    bool Constraints_Base::isActivated()  const{return m_is_activated;};

    const Matrix_h & Constraints_Base::matrix()  const {  return m_A;}
    Matrix_h & Constraints_Base::matrix() { return m_A;}
    unsigned int Constraints_Base::rows() const{return (unsigned int) m_A.rows();};
    unsigned int Constraints_Base::cols() const{return (unsigned int) m_A.cols();};
    
    bool Constraints_Base::setBlock(ConstRefMatrix_h A_block){

        m_A.middleCols(m_block_cols_start, m_block_cols_num) = A_block;
        return true; 
    }
}

