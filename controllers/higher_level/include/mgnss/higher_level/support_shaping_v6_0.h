#ifndef __MGNSS_HIGHER_LEVEL_SUPPORT_SHAPING_V6_0_H
#define __MGNSS_HIGHER_LEVEL_SUPPORT_SHAPING_V6_0_H

#include "mgnss/higher_level/support_shaping_v4_0.h"
#include "mwoibn/robot_class/robot.h"
// #include "mwoibn/hierarchical_control/tasks/controller_task.h"

// #include "mwoibn/robot_points/point.h"
// #include "mwoibn/robot_points/handler.h"

// #include "mwoibn/robot_points/torus_model.h"
#include "mwoibn/dynamic_points/torus.h"
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

/*
 *  Steering version with contact point open-loop reference
 *  For now, just hardcode everythong
 */
class SupportShapingV6: public QrTask
{

public:
  SupportShapingV6(mwoibn::robot_class::Robot& robot, YAML::Node config, std::vector<std::unique_ptr<mwoibn::robot_points::Rotation>>& steering_frames, const mgnss::higher_level::Limit& margin, const mgnss::higher_level::Limit& workspace, mwoibn::robot_points::Handler<mwoibn::dynamic_points::Torus>& accelerations );

  ~SupportShapingV6(){}

  void init(){
    _original_task.init();
    _allocate();
  }


  void _update();

  void log(mwoibn::common::Logger& logger){}

protected:
  virtual void _outputTransform();
  // const mwoibn::VectorN& _spv_desired, &_beta_desired;
  virtual void _allocate();
  mwoibn::robot_points::Handler<mwoibn::dynamic_points::Torus>& _accelerations;
  mwoibn::robot_class::Robot& _robot;
  mgnss::higher_level::SupportShapingV4 _original_task;
  mwoibn::Matrix3 _support_jacobian;
  mwoibn::Vector3 _support_offset;
  mwoibn::Matrix _jacobian;
  mwoibn::VectorN _offset;

};
}
}
#endif
