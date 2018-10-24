#ifndef __MWOIBN__COMMUNICATION_MODULES__POINT_FEEDBACK_H
#define __MWOIBN__COMMUNICATION_MODULES__POINT_FEEDBACK_H

#include "mwoibn/communication_modules/basic_point.h"

#include <rbdl/rbdl.h>

namespace mwoibn
{

namespace communication_modules
{

class PointFeedback : public BasicPoint
{

public:
  PointFeedback(mwoibn::point_handling::Wrench& point)
      : BasicPoint(point)
  {  }

  PointFeedback(mwoibn::point_handling::Wrench& point, YAML::Node config)
      : BasicPoint(point, config)
  {  }

  virtual ~PointFeedback() {}

  virtual bool get() = 0;

  virtual bool update() { get(); }
};
}
}

#endif // BASIC_CONTROLLER_H
