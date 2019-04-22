#ifndef __MWOIBN__DYNAMIC_POINTS__TORUS_H
#define __MWOIBN__DYNAMIC_POINTS__TORUS_H

#include "mwoibn/dynamic_points/dynamic_point.h"
#include "mwoibn/point_handling/frame_plus.h"
#include "mwoibn/point_handling/linear_velocity.h"
#include "mwoibn/point_handling/linear_acceleration.h"
#include "mwoibn/dynamic_models/basic_model.h"
#include "mwoibn/robot_points/torus_model.h"

namespace mwoibn
{

namespace dynamic_points
{

  // Computes the point force given desired accleration
class Torus: public DynamicPoint
{

public:


  Torus(robot_points::TorusModel& torus, robot_class::Robot& robot):
    DynamicPoint(torus._model, torus._state), _torus(torus), _robot(robot){
          _init();
  }


    Torus(robot_points::TorusModel& torus, const mwoibn::robot_class::State& state, robot_class::Robot& robot):
      DynamicPoint(torus._model, torus._state), _torus(torus), _robot(robot){
            _init();
    }


  Torus( Torus&& other)
      : DynamicPoint(other), _torus(other._torus), _robot(other._robot)
  {
    _init();
  }

  Torus(const Torus& other)
      : DynamicPoint(other), _torus(other._torus), _robot(other._robot)
  {
    _init();
  }

  virtual ~Torus() {
  }

  using Point::operator=;

    virtual void compute();

    virtual void computeJacobian();

    const mwoibn::Matrix& getDependant(){return _dependend;}
    const mwoibn::VectorN& getIndependant(){return _independend;}
    const mwoibn::Vector3& getVelocity(){return last_;}
    const mwoibn::Vector3& getEstimate(){return est_;}
    robot_points::TorusModel& torus(){return _torus;}

protected:
  robot_points::TorusModel& _torus;
  robot_class::Robot& _robot;
  mwoibn::Matrix _dependend;
  mwoibn::VectorN _independend;
  mwoibn::Vector3 est_, last_;
  void _init(){
    _dependend.setZero(rows(),3);
    _independend.setZero(rows());
  }

};

} // namespace package
} // namespace library

#endif
