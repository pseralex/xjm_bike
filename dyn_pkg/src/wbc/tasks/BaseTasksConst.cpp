#include "wbc/tasks/BaseTasksConst.h"

namespace wbc_controller_xjm
{
    BaseTasksConst::BaseTasksConst(const std::string & name)
    :m_name(name)
    {
    }
    BaseTasksConst::~BaseTasksConst(){}

    const std::string & BaseTasksConst::name() const
    {
      return m_name;
    }

    void BaseTasksConst::setName(const std::string & name)
    {
      m_name = name;
    }
}
