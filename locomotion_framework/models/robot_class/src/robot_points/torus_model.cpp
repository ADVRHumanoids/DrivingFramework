#include "mwoibn/robot_points/torus_model.h"


  mwoibn::robot_points::TorusModel::TorusModel(RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state, mwoibn::point_handling::FramePlus& centre,
             mwoibn::Axis axis, double r, double R, const mwoibn::Vector3& ground_normal):
                State(model, state), _centre(centre), _axis(axis), _r(r), _R(R), _ground_normal(ground_normal), _v_centre(_centre){

    _point.setZero(3);
    _contact_j.setZero(3, _state.velocity.size());
    _contact_k.setZero(3, _state.velocity.size());

  }

  mwoibn::robot_points::TorusModel::TorusModel(mwoibn::robot_class::Robot& robot, mwoibn::point_handling::FramePlus centre,
             mwoibn::Axis axis, double r, double R, const mwoibn::Vector3& ground_normal):
                State(robot.getModel(), robot.state), _centre(centre), _axis(axis), _r(r), _R(R), _ground_normal(ground_normal), _v_centre(_centre){

    _point.setZero(3);
    _contact_j.setZero(3, _state.velocity.size());
    _contact_k.setZero(3, _state.velocity.size());

  }


  mwoibn::robot_points::TorusModel::TorusModel(RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state, mwoibn::point_handling::FramePlus centre,
             mwoibn::Axis axis, double r, double R, const mwoibn::Vector3& ground_normal):
                State(model, state), _centre(centre), _axis(axis), _r(r), _R(R), _ground_normal(ground_normal),_v_centre(_centre){


    _point.setZero(3);
    _contact_j.setZero(3, _state.velocity.size());
    _contact_k.setZero(3, _state.velocity.size());

  }

  mwoibn::robot_points::TorusModel::TorusModel(TorusModel&& other) : State(other), _centre(other._centre), _v_centre(other._v_centre, _centre), _frame(other._frame),
    _ground_normal(other._ground_normal), _axis(other._axis), _axis_world(other._axis_world), _contact_1(other._contact_1), _contact_2(other._contact_2), _contact_3(other._contact_3),
    _contact_j(other._contact_j), _contact_k(other._contact_k), _temp(other._temp), _r(other._r), _R(other._R){   }

  mwoibn::robot_points::TorusModel::TorusModel(const TorusModel& other) : State(other), _centre(other._centre), _v_centre(other._v_centre, _centre), _frame(other._frame),
    _ground_normal(other._ground_normal), _axis(other._axis), _axis_world(other._axis_world), _contact_1(other._contact_1), _contact_2(other._contact_2), _contact_3(other._contact_3),
    _contact_j(other._contact_j), _contact_k(other._contact_k), _temp(other._temp), _r(other._r), _R(other._R){}

  void mwoibn::robot_points::TorusModel::compute(){

    // std::cout << "centre" << _centre.position.getWorld() << std::endl;
        _axis_world = _centre.rotation().getWorld() * _axis;
       _point = _centre.position.getWorld() + _positionOffset();
    // std::cout << "torus_model" << _point << std::endl;
  }

  void mwoibn::robot_points::TorusModel::computeJacobian() {
    _jacobian.noalias() =  _v_centre.linear().getJacobian();
    _jacobian.noalias() += _jacobianOffset();
  }

  const mwoibn::Vector3& mwoibn::robot_points::TorusModel::_positionOffset(){
      double norm = 1/(_ground_normal - _axis_world*_ground_normal.transpose()*_axis_world).norm();
      _temp = -(_ground_normal - _axis_world*_ground_normal.transpose()*_axis_world)*norm*_R;
      _temp -= (_ground_normal)*_r;
      return _temp;
  }

  const mwoibn::Matrix& mwoibn::robot_points::TorusModel::_jacobianOffset(){

  	  double scalar = _ground_normal.transpose()*_axis_world;
  	  _temp = _axis_world*scalar;

      double norm = 1/(_ground_normal - _temp).norm();


      mwoibn::eigen_utils::skew(_temp, _contact_1);
      mwoibn::eigen_utils::skew(_axis_world, _contact_2);

  	  _contact_3 = _axis_world*_ground_normal.transpose();
      _contact_1 += (_contact_3*_contact_2);

      _contact_2 = 0.5*norm*norm*_ground_normal*_ground_normal.transpose();
      _contact_2 -= mwoibn::Matrix3::Identity();

  	  _contact_2 -= 0.5*norm*norm*_temp*_ground_normal.transpose();
  	  _contact_3 = _contact_2*_contact_1;

      _contact_k.noalias() = _v_centre.angular().getJacobian()*_R*norm;
  	  _contact_j.noalias() = _contact_3*_contact_k;

      return _contact_j;
  }


  const mwoibn::Matrix3& mwoibn::robot_points::TorusModel::frame(){

    _frame.col(0) = _axis_world.cross(_ground_normal).normalized();
    _frame.col(1) = _ground_normal.cross(_frame.col(0));
    _frame.col(2) = _ground_normal;

    return _frame;
  }
