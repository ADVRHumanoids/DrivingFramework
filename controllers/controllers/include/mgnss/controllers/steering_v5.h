#ifndef PROGRAM_STEERING_V5_H
#define PROGRAM_STEERING_V5_H

#include <mwoibn/robot_class/robot.h>
#include <mwoibn/hierarchical_control/cartesian_simplified_pelvis_task_v5.h>

namespace mgnss
{

namespace events
{

// void limit(double b_ref, double& b);
// void limit(const mwoibn::VectorN& b_ref, mwoibn::VectorN& b);
// void jointLimits(double& b, double max = 2.79252680);
// void jointLimits(mwoibn::VectorN& b, double max = 2.79252680);

class Steering5
{

public:
  Steering5(mwoibn::robot_class::Robot& robot,
            mwoibn::hierarchical_control::CartesianFlatReferenceTask2& plane,
            mwoibn::VectorN init_pose, double K_icm, double K_sp, double dt,
            double margin = 0.04, double max = 2.79252680);

  ~Steering5() {}

  const mwoibn::VectorN& get() { return _b_st; }
  void set(mwoibn::VectorN last) { _b_st = last; }

  void setRate(double dt)
  {
    _dt = dt;
    _treshhold = _margin / _dt;
  }
  //  void compute(const mwoibn::Vector3 next_step);
  void compute2(const mwoibn::Vector3 next_step);
  const mwoibn::VectorN& getICM() { return _b_icm; }
  const mwoibn::VectorN& getSP() { return _b_sp; }
  const mwoibn::VectorN& vICM() { return _v_icm; }
  const mwoibn::VectorN& vSP() { return _v_sp; }
  const mwoibn::VectorN& getDampingSP() { return _damp_sp; }
  const mwoibn::VectorN& getDampingICM() { return _damp_icm; }
  void resteer(const mwoibn::VectorBool& steer)
  {
    for (int i = 0; i < _resteer.size(); i++)
    {
//      if (!_steer[i])
        _resteer[i] = steer[i];
//      else
//        _resteer[i] = false;
    }
//    _steer.noalias() = steer;
  }

  static void limit2PI(double ref, double& st);
  static void limitPI(double ref, double& st);

protected:
  mwoibn::hierarchical_control::CartesianFlatReferenceTask2& _plane;
  double _dt, _margin, _max, _K_icm, _K_sp, _heading, _x, _y, _treshhold;
  mwoibn::VectorN _v_icm, _b_icm, _v_sp, _b_sp, _b, _b_st, _plane_ref, _damp_sp,
      _damp_icm, _pb_icm, _pb_sp;
  const mwoibn::VectorN& _state;
  mwoibn::VectorInt _dofs;
  mwoibn::VectorBool _resteer, _steer;
  int _size = 4;

  void _ICM(mwoibn::Vector3 next_step);

  void _SPT();

  void _PT(int i);
};
}
}
#endif // PROGRAM_STEERING_H
