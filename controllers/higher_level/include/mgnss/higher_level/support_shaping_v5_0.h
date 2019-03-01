#ifndef __MGNSS_HIGHER_LEVEL_SUPPORT_SHAPING_V5_0_H
#define __MGNSS_HIGHER_LEVEL_SUPPORT_SHAPING_V5_0_H

#include "mgnss/higher_level/support_shaping_v4_0.h"
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

/*
 *  Steering version with contact point open-loop reference
 *  For now, just hardcode everythong
 */
class SupportShapingV5: public SupportShapingV4
{

public:
  SupportShapingV5(mwoibn::robot_class::Robot& robot, YAML::Node config, std::vector<std::unique_ptr<mwoibn::robot_points::Rotation>>& steering_frames, const mgnss::higher_level::Limit& margin, const mgnss::higher_level::Limit& workspace, const mwoibn::VectorN& spv, const mwoibn::VectorN& beta);

  ~SupportShapingV5(){}


  // void init();
  void _update();

  // void log(mwoibn::common::Logger& logger);

protected:
  virtual void _outputTransform();
  const mwoibn::VectorN& _spv_desired, &_beta_desired;
  virtual void _allocate();

};
}
}
#endif
