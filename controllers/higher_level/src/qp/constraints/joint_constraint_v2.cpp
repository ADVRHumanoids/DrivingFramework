#include "mgnss/higher_level/qp/constraints/joint_constraint_v2.h"

mgnss::higher_level::constraints::JointConstraintV2::JointConstraintV2(const JointConstraintV2& other):
    Constraint(other), _robot(other._robot), _interfaces(other._interfaces), _gravity(other._gravity) {
  _init(_interfaces);

}


mgnss::higher_level::constraints::JointConstraintV2::JointConstraintV2(mwoibn::robot_class::Robot& robot,
                                    const mwoibn::VectorInt& dofs, std::vector<mwoibn::Interface> interfaces, mwoibn::dynamic_models::BasicModel& gravity):
                                    Constraint(2*dofs.size(), robot.getDofs()), _robot(robot),  _interfaces(interfaces), _gravity(gravity){
  active_dofs = dofs;
  _init(interfaces);
}

void mgnss::higher_level::constraints::JointConstraintV2::_init(std::vector<mwoibn::Interface>& interfaces){
  _gravity.subscribe({mwoibn::dynamic_models::DYNAMIC_MODEL::INERTIA});

  _support_jacobian.setIdentity(_robot.getDofs(), _robot.getDofs());

  // std::cout << __PRETTY_FUNCTION__ << "\t" << dofs.transpose() << std::endl;
  if(!interfaces.size())
    throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": Could not initalize joint space constraints. Received an empty interface vector. Please, specify a robot interface."));

    if(ranges::find(interfaces, "POSITION") != ranges::end(interfaces)) _initPositions();
    if(ranges::find(interfaces, "VELOCITY") != ranges::end(interfaces)) _initVelocities();
    if(ranges::find(interfaces, "TORQUE") != ranges::end(interfaces)) _initTorques();

    _min.active_dofs = active_dofs;
    _max.active_dofs = active_dofs;

    _min.active.setConstant(_robot.getDofs(),false);
    _max.active.setConstant(_robot.getDofs(),false);
    // define _min.active & _max.active
    auto dofs_ = mwoibn::eigen_utils::toVector<unsigned int>(active_dofs);
    for (auto& dof: dofs_){
      _min.active[dof] = true;
      _max.active[dof] = true;
    }

    mwoibn::VectorBool temp_(_min.active.size());
    temp_.setConstant(false);
    for(auto& constraint: _min){
      mwoibn::eigen_utils::boolOr(temp_, constraint->active, temp_);
    }
    mwoibn::eigen_utils::boolAnd(_min.active, temp_, _min.active);


    temp_.setConstant(false);
    for(auto& constraint: _max)
      mwoibn::eigen_utils::boolOr(temp_, constraint->active, temp_);

    mwoibn::eigen_utils::boolAnd(_max.active, temp_, _max.active);

    if(_torque_min && _torque_max)
      resize(_min.active.count() + _max.active.count() + _torque_max->active.count() + _torque_min->active.count(), _robot.getDofs());
    else
      resize(_min.active.count() + _max.active.count(), _robot.getDofs());
    _min.init();
    _max.init();

    _inactive_dofs.setZero(_min.active.count());
    for(int i = 0, k = 0; i < _min.active.size(); i++){
        if(_min.active[i]){
        _inactive_dofs[k] = i;
        ++k;
      }
    }

    _test_inactive.setZero(_robot.getDofs());
    update();
}


void mgnss::higher_level::constraints::JointConstraintV2::update(){
        _max_position = -_robot.state.position.get();
        _max_velocity = -_robot.state.velocity.get();

        _min.update();
        _max.update();

        _state.head(_min.active.count()) = _min.getState();
        _state.segment(_min.active.count(), _max.active.count()) = _max.getState();
        _jacobian.topRows(_min.active.count()) = _min.getJacobian();
        _jacobian.block(_min.active.count(), 0, _max.active.count(), _robot.getDofs()) = _max.getJacobian();

        if(!_torque_min || !_torque_max) return;

        // _gravity.update();


      //   std::cout << __PRETTY_FUNCTION__ << std::endl;
      // for (int i = 0; i < _robot.getDofs(); i++){
      //   std::cout << _robot.state["BIAS_FORCE"].get()[i] << "\t";
      //   std::cout << _robot.state.torque.get()[i] << "\t";
      //   std::cout << _robot.lower_limits["BIAS_FORCE"].get()[i] << "\t";
      //   std::cout << _robot.upper_limits["BIAS_FORCE"].get()[i] << std::endl;
      // }
        _min_torque =  _robot.state.velocity.get();
        _max_torque = -_robot.state.velocity.get();


        _test_inactive.setZero();
        // for (int i = 0; i < _inactive_dofs.size(); i++){
        //     _test_inactive += _gravity.getInertia().col(_inactive_dofs[i])*_robot.state.acceleration.get()[_inactive_dofs[i]]/_robot.rate();
        // }
        // for (int i = 0; i < _inactive_dofs.size(); i++){
        //     _test_inactive += _gravity.getInertia().col(_inactive_dofs[i])*_robot.state.acceleration.get()[_inactive_dofs[i]]/_robot.rate();
        // }


        _robot.lower_limits["BIAS_FORCE"].set(_robot.state["BIAS_FORCE"].get() + _robot.lower_limits.torque.get() + _test_inactive);
        _robot.upper_limits["BIAS_FORCE"].set(_robot.state["BIAS_FORCE"].get() + _robot.upper_limits.torque.get() + _test_inactive);

        // std::cout << _test_inactive.transpose() << std::endl;
        // std::cout << _robot.state["BIAS_FORCE"].get().transpose() << std::endl;


        // std::cout << ""
        _torque_max->update();
        _torque_min->update();

        int k = _min.active.count()+_max.active.count();
        for(int i = 0; i < _torque_min->active.size(); i++){
          if(_torque_min->active[i]){
              _jacobian.row(k) = _torque_min->getJacobian().row(i);
              _state[k] = _torque_min->getState()[i];
              if (_torque_min->getState()[i] < 0)
                std::cout << "min k: " << k << ", i: " << i << "\t" << _torque_min->getState()[i]  << "\t" << _robot.state["BIAS_FORCE"].get()[i]*_robot.rate() << "\t" <<  _robot.lower_limits["BIAS_FORCE"][i]*_robot.rate() << "\t" << (_torque_min->getJacobian()*_min_torque)[i] << std::endl;
              ++k;
          }
        }

        for(int i = 0; i < _torque_max->active.size(); i++){
          if(_torque_max->active[i]){
              _jacobian.row(k) = _torque_max->getJacobian().row(i);
              _state[k] = _torque_max->getState()[i];
              if (_torque_max->getState()[i] < 0)
                std::cout << "max k: " << k << ", i: " << i << "\t" << _torque_max->getState()[i] << "\t" << _robot.state["BIAS_FORCE"].get()[i]*_robot.rate() << "\t" <<  _robot.upper_limits["BIAS_FORCE"][i]*_robot.rate() << "\t" << (_torque_max->getJacobian()*_max_torque)[i]<< std::endl;
              ++k;
          }
        }

        // std::cout << "state\n" << _state.transpose() << std::endl;

        // std::cout << "state\n" << _state.tail(_torque_max->active.count()+_torque_min->active.count()).transpose() << std::endl;
        // std::cout << "torque_active\n" << _torque_min->active.transpose() << std::endl;

        // std::cout << "jacobian\n" << _jacobian << std::endl;
}

void mgnss::higher_level::constraints::JointConstraintV2::_init(mwoibn::Interface interface, mgnss::higher_level::Constraint& min, mgnss::higher_level::Constraint& max){

  min.active_dofs = active_dofs;
  max.active_dofs = active_dofs;
  min.active.setConstant(false);
  max.active.setConstant(false);
  auto dofs_ = mwoibn::eigen_utils::toVector<unsigned int>(active_dofs);
  for (auto& dof: dofs_){
    min.active[dof] = true;
    max.active[dof] = true;
  }

  // init basic constraint
  for(int i = 0; i < _robot.getDofs(); i++){
    // std::cout << _robot.lower_limits[interface].get()[i] << "\t" <<
    //             (_robot.lower_limits[interface].get()[i] != mwoibn::NON_EXISTING) << std::endl;
    min.active[i] = min.active[i] && (_robot.lower_limits[interface].get()[i] != mwoibn::NON_EXISTING);
    max.active[i] = max.active[i] && (_robot.upper_limits[interface].get()[i] != mwoibn::NON_EXISTING);
  }
  // std::cout << "min\t" << min.active.transpose() << std::endl;
  // std::cout << "max\t" << max.active.transpose() << std::endl;

  min.init();
  max.init();
}
