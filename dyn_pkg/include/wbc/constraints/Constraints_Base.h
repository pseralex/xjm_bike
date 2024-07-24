#ifndef CONSTRAINTS_BASE_H
#define CONSTRAINTS_BASE_H
#pragma once
#include "wbc/fwd/fwd.hpp"

namespace wbc_controller_xjm
{
  class Constraints_Base
  {
  public:
    /// cols 列数即为所有决策变量的个数
    Constraints_Base(const unsigned int rows, const unsigned int block_cols_start, const unsigned int block_cols_num, const unsigned int cols=55);
    virtual ~Constraints_Base() {}

    virtual bool isTarget() const;
    virtual bool isEquality() const;
    virtual bool isInequality() const;
    virtual bool isBound() const;
    virtual bool isZero() const;

    virtual void setActivated(bool is_activated); /// 设置是否使能
    virtual bool isActivated() const;

    virtual const Matrix_h & matrix() const;
    virtual Matrix_h & matrix();
    virtual unsigned int rows() const;
    virtual unsigned int cols() const;

    /// 
    virtual bool setVector(ConstRefVector_h b) = 0;
    virtual bool setLowerBound(ConstRefVector_h lb) = 0;
    virtual bool setUpperBound(ConstRefVector_h ub) = 0;

    virtual const Vector_h & vector() const = 0;
    virtual const Vector_h & lowerBound() const = 0;
    virtual const Vector_h & upperBound() const = 0;
    virtual Vector_h & vector() = 0;
    virtual Vector_h & lowerBound() = 0;
    virtual Vector_h & upperBound() = 0;

    virtual bool setBlock(ConstRefMatrix_h A_block);

  protected: 
    Matrix_h m_A;
    int      m_block_cols_start;
    int      m_block_cols_num;

    bool m_is_target      = false;
    bool m_is_equality    = false;
    bool m_is_zero        = false;
    bool m_is_inEquality  = false;
    bool m_is_bound       = false;
    bool m_is_activated   = true;  /// 默认都是激活的

  private:
    virtual void initialize();

  }; 
} 
#endif