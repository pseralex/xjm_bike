#include "wbc/trajes/TrajEuc.hpp"

namespace wbc_controller_xjm
{
    TrajEuc::TrajEuc(unsigned int size)
    {
        resize(size);
    }
    TrajEuc::~TrajEuc(){};

    void TrajEuc::resize(unsigned int size)
    {
        m_pos.setZero(size);
        m_vel.setZero(size);
        m_acc.setZero(size);
    }

    const Vector_h & TrajEuc::getPos() const { return m_pos; }
    const Vector_h & TrajEuc::getVel() const { return m_vel; }
    const Vector_h & TrajEuc::getAcc() const { return m_acc; }

    void TrajEuc::setPos(const Vector_h & pos) { m_pos = pos; }
    void TrajEuc::setVel(const Vector_h & vel) { m_vel = vel; }
    void TrajEuc::setAcc(const Vector_h & acc) { m_acc = acc; }

    void TrajEuc::setTrajEuc(const TrajEuc & trajEuc){
        m_pos = trajEuc.getPos();
        m_vel = trajEuc.getVel();
        m_acc = trajEuc.getAcc();
    }

}
