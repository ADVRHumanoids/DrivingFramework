#ifndef __MWOIBN__DYNAMIC_POINTS__TORUS_INTERGRATED_ROLL_H
#define __MWOIBN__DYNAMIC_POINTS__TORUS_INTERGRATED_ROLL_H

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

  // Computes the torus acceleration
class TorusIntegratedRoll: public DynamicPoint
{

public:


  TorusIntegratedRoll(robot_points::TorusModel& torus, robot_class::Robot& robot):
    DynamicPoint(torus._model, torus._state), acceleration(torus, robot), _torus(torus), _robot(robot){
          _init();
  }


    TorusIntegratedRoll(robot_points::TorusModel& torus, const mwoibn::robot_class::State& state, robot_class::Robot& robot):
      DynamicPoint(torus._model, torus._state), acceleration(torus, robot), _torus(torus), _robot(robot){
            _init();
    }


  TorusIntegratedRoll( TorusIntegratedRoll&& other)
      : DynamicPoint(other), acceleration(other._torus, other._robot), _torus(other._torus), _robot(other._robot)
  {
    _init();
  }

  TorusIntegratedRoll(const TorusIntegratedRoll& other)
      : DynamicPoint(other), acceleration(other._torus, other._robot), _torus(other._torus), _robot(other._robot)
  {
    _init();
  }

  virtual ~TorusIntegratedRoll() {
  }

  using Point::operator=;

    virtual void compute(){
      //  mwoibn::Matrix3 temp_1_ = -acceleration.getJacobianWheel() + acceleration.getJacobianVelocity()*_robot.rate();
        mwoibn::Matrix3 temp_1_ = -acceleration.getJacobianWheel() + acceleration.getJacobianVelocity()*_robot.rate();
        //std::cout << "acceleration\t" << (acceleration.getJacobianWheel() - torus().getJacobianWheel()) << std::endl;
        //std::cout << "velocity\t" << (acceleration.getJacobianVelocity()*_robot.rate()) << std::endl;
        mwoibn::Matrix3 temp_2_ = temp_1_*_null_space;
        temp_2_ += torus().getJacobianWheel();
        _constant = temp_2_*_torus._v_centre.angular().getWorld();
        //std::cout << "1\t" << temp_1_ << std::endl;
        // std::cout << "2\t" << torus().getJacobianWheel() << std::endl;

//        std::cout << "_constant\t" << _constant.transpose() << std::endl;

        //_point = torus().get();
        // _point = torus().getJacobianWheel()*_torus._v_centre.angular().getWorld();
        // _point += _wheel_jacobian*_torus._v_centre.angular().getWorld()*_robot.rate();
        // _point += _constant;
    }



    virtual void computeJacobian(){
        acceleration.update(true);
        _normal_projection = _torus.groundNormal()*_torus.groundNormal().transpose();
        _null_space = mwoibn::Matrix3::Identity() - _normal_projection;
        _wheel_jacobian = acceleration.getJacobianWheel()*_null_space;
        _wheel_jacobian += acceleration.getJacobianVelocity()*_normal_projection*_robot.rate();
        _jacobian = _wheel_jacobian*_torus._v_centre.angular().getJacobian();
    }
    //
    // const mwoibn::Matrix& getDependant(){return _dependend;}
    // const mwoibn::VectorN& getIndependant(){return _independend;}
    // const mwoibn::Vector3& getVelocity(){return last_;}
    // const mwoibn::Vector3& getEstimate(){return est_;}
    robot_points::TorusModel& torus(){return _torus;}
    dynamic_points::TorusRoll acceleration;
    const mwoibn::Matrix3& getJacobianWheel(){return _wheel_jacobian;}

    virtual void update(bool jacobian) {
          acceleration.update(true);
          if(jacobian)
            computeJacobian();
          compute();
    }

protected:
  robot_points::TorusModel& _torus;
  robot_class::Robot& _robot;
  mwoibn::Matrix3 _wheel_jacobian;
  mwoibn::Matrix3 _velocity_jacobian;//??
  mwoibn::Matrix3 _normal_projection;
  mwoibn::Matrix3 _null_space;
  //mwoibn::Matrix _dependend;
  //mwoibn::VectorN _independend;
  // mwoibn::Vector3 est_, last_;
  void _init(){
    // _dependend.setZero(rows(),3);
    // _independend.setZero(rows());
  }

};

} // namespace package
} // namespace library

#endif
