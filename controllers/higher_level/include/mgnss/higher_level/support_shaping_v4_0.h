#ifndef __MGNSS_HIGHER_LEVEL_SUPPORT_SHAPING_V4_0_H
#define __MGNSS_HIGHER_LEVEL_SUPPORT_SHAPING_V4_0_H

#include "mgnss/higher_level/qr_task.h"
#include "mwoibn/robot_class/robot.h"
// #include "mwoibn/hierarchical_control/tasks/controller_task.h"

// #include "mwoibn/robot_points/point.h"
// #include "mwoibn/robot_points/handler.h"

// #include "mwoibn/robot_points/torus_model.h"
// #include "mwoibn/robot_points/linear_point.h"
// #include "mwoibn/robot_points/minus.h"
// #include "mwoibn/robot_points/rotation.h"
// #include "mwoibn/robot_points/ground_wheel.h"

// #include "mwoibn/common/logger.h"

#include "mgnss/higher_level/state_machine.h"

namespace mgnss
{

namespace higher_level
{


  class SupportShapingV6;
/*
 *  Steering version with contact point open-loop reference
 *  For now, just hardcode everythong
 */
class SupportShapingV4: public QrTask
{

public:
  SupportShapingV4(mwoibn::robot_class::Robot& robot, YAML::Node config, std::vector<std::unique_ptr<mwoibn::robot_points::Rotation>>& steering_frames, const mgnss::higher_level::Limit& margin, const mgnss::higher_level::Limit& workspace, bool is_margin = true);

  ~SupportShapingV4(){}

  friend class SupportShapingV6;


  void init(){    _allocate();}
  void _update();

  void log(mwoibn::common::Logger& logger);

protected:
  mwoibn::robot_class::Robot& _robot;
  unsigned int _size;
  mwoibn::VectorN _safety, _max_workspace;
  bool _is_margin;
  const mgnss::higher_level::Limit &_margin, &_workspace;

  std::vector<std::unique_ptr<mwoibn::robot_points::Rotation>>& _wheel_transforms;

  virtual void _outputTransform();
  virtual void _allocate();
};
}
}
#endif
