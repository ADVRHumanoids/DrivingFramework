//#ifndef HIERARCHICAL_CONTROL_CAMBER_ANGLE_TASK_H
//#define HIERARCHICAL_CONTROL_CAMBER_ANGLE_TASK_H

//#include "mwoibn/hierarchical_control/hierarchical_control.h"
//#include "mwoibn/hierarchical_control/controller_task.h"
//#include "mwoibn/point_handling/robot_points_handler.h"
//#include <rbdl/rbdl.h>

//namespace mwoibn
//{
//namespace hierarchical_control
//{

///**
// * @brief The CartesianWorldTask class Provides the inverse kinematics task
// *to control the position of a point defined in one of a robot reference frames
// *
// */

//class CamberAngle
//{
//public:
//  CamberAngle(mwoibn::robot_class::Robot& robot,
//              mwoibn::point_handling::Point point, mwoibn::Axis x,
//              mwoibn::Axis y, mwoibn::Axis z)
//      : _x_body(x), _y_body(y), _z_body(z), _point(point), _robot(robot)
//  {
//    _x_world << 1,0,0;
//    _y_world << 0,1,0;
//    _z_world << 0,0,1;

//  } // for now just support major axes

//  ~CamberAngle() {}

//  void update()
//  {
//    _y = _point.getRotationWorld(_robot.state.get()) *
//        _y_world;

//    std::cout << "_y\t" << _y.transpose() << std::endl;
//    _n = _y.cross(_z_world);
////    std::cout << "_n\t" << _n.transpose() << std::endl;
//    double b = 1/_n.norm();

//    _n.normalize();
//   std::cout << "_n norm\t" << _n.transpose() << std::endl;

//    _v1 = _z_world - _y * _z_world.transpose() * _y;
//    std::cout << "_v1\t" << _v1.transpose() << std::endl;

////    std::cout << "b\t" << b << std::endl;

//    _v1.normalize();

////    std::cout << "_v1 norm\t" << _v1.transpose() << std::endl;

//    _v2 = _z_world;
//    std::cout << "_v2\t" << _v2.transpose() << std::endl;

////    double b = 1/_v2.norm();
////    _v2.normalize();

//    double cross = (_v1.cross(_v2)).transpose() * _n;
//    double dot = _v1.transpose() * _v2;
//    _camber = std::atan2(cross, dot);
////    std::cout << "cross\t" << cross << std::endl;
////    std::cout << "dot\t" << dot<< std::endl;

//    std::cout << "_camber\t" << _camber*180/3.14 << std::endl;
//// DERIVATIVE

//    double H = cross*cross + dot * dot;

//    double I = dot / H;
//    H = cross / H;

//    mwoibn::Matrix C = -(0.5*b*b*_z_world*_z_world.transpose() - mwoibn::Matrix::Identity(3,3) - 0.5*b*b*_y*_y.transpose()*_z_world*_z_world.transpose())*b;
//    mwoibn::Matrix c = _skew(_y*_z_world.transpose()*_y) + _y*_z_world.transpose()*_skew(_y);
//    C = C*c;


//    mwoibn::Matrix J = -0.5*b*b*b * _z_world.transpose()*c;
//    mwoibn::Matrix K = _skew(_y)*_v2*J + b*_skew(_v2)*_skew(_y);
//    mwoibn::Matrix L = _v2.transpose()*C;
//    mwoibn::Matrix M = -_n.transpose()*_skew(_v2)*C - (_skew(_v2)*_v1).transpose()*K;

//    mwoibn::Matrix3 R;
//    R.col(0) = _x_body;
//    R.col(1) = _y_body;
//    R.col(2) = _z_body;

//    _J = (H*L + I*M) * R* _point.getOrientationJacobian(_robot.state.get());
////    std::cout << "(H*L + I*M)\n" << (H*L + I*M) << std::endl;
//////    std::cout << "J point\n" <<  _point.getOrientationJacobian(_robot.state.get()) << std::endl;
//    std::cout << "camber J\n" <<  _J << std::endl;

//  }

//  double get(){
//    return _camber;}
//  const mwoibn::Matrix& getJacobian(){return _J;}

//protected:
//  mwoibn::Axis _x_body, _y_body, _z_body;
//  mwoibn::Axis _v2, _n, _v1, _y;

//  mwoibn::Axis _x_world, _y_world,
//      _z_world; // for now assume the basic initialization
//  double _camber, _d_camber;
//  mwoibn::point_handling::Point _point;
//  mwoibn::robot_class::Robot& _robot;

//  mwoibn::Matrix _J;
//  mwoibn::Matrix3 _skew(mwoibn::Vector3 vec){

//    mwoibn::Matrix3 mat;
//    mat << 0, -vec[2], vec[1], vec[2], 0, -vec[0], -vec[1], vec[0], 0;

//    return mat;
//  }
//};

// class CamberAngleTask : public ControllerTask
// {

// public:
//  CamberAngleTask(std::vector<hierarchical_control::CamberAngle> angels, mwoibn::robot_class::Robot& robot)
//      : ControllerTask(), _robot(robot), _angels(angels)
//  {
//    _init(_angels.size(), _robot.getDofs());
//    _ref.setZero(_angels.size());
//  }

//  virtual ~CamberAngleTask() {}

//  virtual void updateError()
//  {
//    _last_error.noalias() = _error;

//    for(int i = 0; i < _angels.size(); i++){
//      _angels[i].update();
//      _error[i] = _ref[i] - _angels[i].get();
//    }
//    eigen_utils::limitToHalfPi(_error); // make a bigger limit to avoid chattering
//    std::cout << std::fixed << "camber error\n" << _error.transpose() * 180 / 3.14 << std::endl;

//  }

//  mwoibn::VectorN getCurrent(){
//    mwoibn::VectorN current;
//    current.setZero(4);

//    for(int i = 0; i < _angels.size(); i++)
//      current[i] = _angels[i].get();

//    return current;
//  }

//  virtual void updateJacobian() {
//    _last_jacobian.noalias() = _jacobian;

//    for(int i = 0; i < _angels.size(); i++){
//      _jacobian.row(i) = _angels[i].getJacobian();
//    }
//  }

//  virtual const mwoibn::VectorN& getReference() const { return _ref; }

//  virtual void setReference(const mwoibn::VectorN& reference)
//  {
//    _ref = reference;
//  }

//  virtual double getReference(int i) const { return _ref[i]; }
//  virtual void setReference(int i, double reference) { _ref[i] = reference; }

// protected:
//  mwoibn::robot_class::Robot& _robot;
//  mwoibn::VectorN _ref, _current;
//  std::vector<hierarchical_control::CamberAngle> _angels;
// };
//} // namespace package
//} // namespace library
//#endif
