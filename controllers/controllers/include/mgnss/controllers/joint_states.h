#ifndef PROGRAMS_JOINT_STATES_H
#define PROGRAMS_JOINT_STATES_H

#include <mwoibn/robot_class/robot.h>
#include <mwoibn/point_handling/robot_points_handler.h>
#include <mgnss/controllers/steering.h>

namespace mgnss
{
namespace controllers
{
class JointStates
{

public:
  JointStates(mwoibn::robot_class::Robot& robot);

  ~JointStates() {}

  bool setPosition(std::string name);
  bool setFullPosition(std::string name);

  void step(double step) { _step = step; }

  void update();
  void send();

protected:
  mwoibn::VectorN _position, _velocity, _last_ankle, _last_position, _des_ankle, _init_ankle;
  mwoibn::VectorN _pos_ref, _vel_ref;
  mwoibn::VectorInt _vel_map, _ankle_map, _vel_sign, _yaw_map;
  mwoibn::Vector3 _error, _last;
  mwoibn::point_handling::PositionsHandler _wheels;
  std::vector<mwoibn::Vector3> _wheels_positions;
  mwoibn::robot_class::Robot& _robot;
  double _step;
  bool _init;
};
}
}

#endif // WHEELED_MOTION_H
