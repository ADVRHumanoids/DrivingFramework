#ifndef PROGRAM_STEERING_H
#define PROGRAM_STEERING_H

#include <mwoibn/robot_class/robot.h>
#include <mwoibn/hierarchical_control/cartesian_simplified_pelvis_task_v3.h>
#include <mgnss/controllers/steering_reference.h>

namespace mgnss {

namespace events
{

class Steering
{

public:
  Steering(mwoibn::robot_class::Robot& robot,
           mwoibn::hierarchical_control::CartesianSimplifiedPelvisTask& plane, mwoibn::VectorN init_pose,
           double K_icm, double K_sp, double dt, double margin = 0.04,
           double max = 2.79252680);

  ~Steering() {}

  const mwoibn::VectorN& get() { return _b_st; }
  void set(mwoibn::VectorN last) {_b_st = last;}

  void compute(const mwoibn::Vector3 next_step);

protected:
  mwoibn::hierarchical_control::CartesianSimplifiedPelvisTask& _plane;
  double _dt, _margin, _max, _K_icm, _K_sp, _heading, _x, _y;
  mwoibn::VectorN _v_icm, _b_icm, _v_sp, _b_sp, _b, _b_st, _plane_ref, _temp;
  const mwoibn::VectorN& _state;
  mwoibn::VectorInt _dofs;
  int _size = 4;

  void _ICM(mwoibn::Vector3 next_step);

  void _SPT();

  void _PT(int i);
};
}
}
#endif // PROGRAM_STEERING_H
