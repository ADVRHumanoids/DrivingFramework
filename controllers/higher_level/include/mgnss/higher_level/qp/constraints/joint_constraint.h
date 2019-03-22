#ifndef __MGNSS_HIGHER_LEVEL_JOINT_CONSTRAINT_H
#define __MGNSS_HIGHER_LEVEL_JOINT_CONSTRAINT_H

#include "mwoibn/robot_class/robot.h"
#include "mgnss/higher_level/qp/constraints/constraint.h"


namespace mgnss
{

namespace higher_level
{

class JointConstraint: public Constraint{

  public:
    JointConstraint(mwoibn::robot_class::Robot& robot, const mwoibn::VectorInt& dofs, std::vector<mwoibn::Interface> interfaces): Constraint(2*dofs.size(), robot.getDofs()), _robot(robot){

      // std::cout << __PRETTY_FUNCTION__ << "\t" << dofs.transpose() << std::endl;
      if(!interfaces.size())
        throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": Could not initalize joint space constraints. Received an empty interface vector. Please, specify a robot interface."));

      for(auto& interface: interfaces){
        if(interface == "POSITION") _position = true;
        if(interface == "VELOCITY") _velocity = true;
      }
      // std::vector<unsigned int> position_dofs, velocity_dofs;
      // if(_position)
      //   position_dofs = mwoibn::eigen_utils::toVector<unsigned int>(dofs) | ranges::action::remove_if([&](int i){return _robot.lower_limits.position.get()[i] == mwoibn::NON_EXISTING || _robot.lower_limits.position.get()[i] == mwoibn::NON_EXISTING;}) | ranges::action::sort;
      // if(_velocity)
      //   velocity_dofs = mwoibn::eigen_utils::toVector<unsigned int>(dofs) | ranges::action::remove_if([&](int i){return _robot.lower_limits.velocity.get()[i] == mwoibn::NON_EXISTING || _robot.lower_limits.velocity.get()[i] == mwoibn::NON_EXISTING;}) | ranges::action::sort;

        // std::merge(position_dofs.begin(), position_dofs.end(), velocity_dofs.begin(), velocity_dofs.end(),_dofs.begin());
        // _dofs |= ranges::action::unique;

        _dofs = mwoibn::eigen_utils::toVector<unsigned int>(dofs) | ranges::action::remove_if([&](int i){return (
            (!_velocity || _robot.lower_limits.velocity.get()[i] == mwoibn::NON_EXISTING || _robot.upper_limits.velocity.get()[i] == mwoibn::NON_EXISTING) &&
            (!_position || _robot.lower_limits.position.get()[i] == mwoibn::NON_EXISTING || _robot.upper_limits.position.get()[i] == mwoibn::NON_EXISTING));});

        resize(2*_dofs.size(), _robot.getDofs());

        for(int i = 0; i < _dofs.size(); i++){
        if(_velocity)
          _velocity_dofs.push_back( _robot.lower_limits.velocity.get()[_dofs[i]] != mwoibn::NON_EXISTING && _robot.lower_limits.velocity.get()[_dofs[i]] != mwoibn::NON_EXISTING);
        if(_position)
          _position_dofs.push_back( _robot.lower_limits.position.get()[_dofs[i]] != mwoibn::NON_EXISTING && _robot.lower_limits.position.get()[_dofs[i]] != mwoibn::NON_EXISTING);
          _jacobian(i,_dofs[i]) = 1; //min
          _jacobian(_dofs.size()+i, _dofs[i]) = -1; // max
        }

      //
      // if(_velocity)
      //   // _position_dofs = ;
      //   _velocity_dofs = mwoibn::eigen_utils::toVector<unsigned int>(dofs) | ranges::action::remove_if([&](int i){return _robot.lower_limits.velocity.get()[i] == mwoibn::NON_EXISTING || _robot.lower_limits.velocity.get()[i] == mwoibn::NON_EXISTING;});



      // _dofs = ranges::view::concat(_position_dofs, _velocity_dofs) | ranges::actions::stable_sort | ranges::views::unique;
      //


    }


    virtual void update(){
      // if(_velocity){

      // std::cout << __PRETTY_FUNCTION__;
      // for(auto& id: _dofs)
      //   std::cout << "\t" << id;
      // std::cout << std::endl;

      for(int i = 0; i < _dofs.size(); i++){
          bool velocity = _velocity && _velocity_dofs[i];
          bool position = _position && _position_dofs[i];
          if( velocity && position){
            _state[i] = std::min(-_robot.lower_limits.velocity.get(_dofs[i]), ( _robot.state.position.get()[_dofs[i]]-_robot.lower_limits.position.get(_dofs[i]))/_robot.rate());
            _state[_dofs.size()+i] = std::min( _robot.upper_limits.velocity.get(_dofs[i]), (-_robot.state.position.get()[_dofs[i]]+_robot.upper_limits.position.get(_dofs[i]))/_robot.rate());
            // std::cout << "mixed\t" << i << "\t" << state[i] << ", " << state[_dofs.size()+i] <<  std::endl;
          }
          else if( velocity){
            _state[i] = -_robot.lower_limits.velocity.get(_dofs[i]);
            _state[_dofs.size()+i] =  _robot.upper_limits.velocity.get(_dofs[i]);
            // std::cout << "velocity\t" << i << "\t" << state[i] << ", " << state[_dofs.size()+i] <<  std::endl;
          }
          else if( position){
            _state[i] = (_robot.state.position.get()[_dofs[i]]-_robot.lower_limits.position.get(_dofs[i]))/_robot.rate();
            _state[_dofs.size()+i] = (-_robot.state.position.get()[_dofs[i]]+_robot.upper_limits.position.get(_dofs[i]))/_robot.rate();
            // std::cout << "position\t" << "\t" << state[i] << ", " << state[_dofs.size()+i] <<  std::endl;
          }

          // std::cout << i << "\t _robot.lower_limits.velocity\t" << -_robot.lower_limits.velocity.get(_dofs[i]);
          // std::cout << "\t _robot.upper_limits.velocity\t" << _robot.upper_limits.velocity.get(_dofs[i]);
          // std::cout << "\t _robot.lower_limits.position\t" <<  (_robot.state.position.get()[_dofs[i]]-_robot.lower_limits.position.get(_dofs[i]))/_robot.rate();
          // std::cout << "\t _robot.upper_limits.position\t" <<  (-_robot.state.position.get()[_dofs[i]]+_robot.upper_limits.position.get(_dofs[i]))/_robot.rate();
          // std::cout << std::endl;
      }

      // auto select =  mwoibn::eigen_utils::toVector<mwoibn::Scalar>(state) | ranges::action::remove_if([](int i){return i > 0;}) ;
      // auto select = mwoibn::eigen_utils::toVector<mwoibn::Scalar>(_state);
      // for(auto&& zip:  ranges::view::zip( _dofs, select )){
      //   if(std::get<1>(zip) < 0){
      //     std::cout << "joint_limits\t" << std::get<0>(zip) << ", " << _robot.getLinks(std::get<0>(zip)) << " " << std::get<1>(zip) << std::endl;
      //     std::cout <<  "\t _robot.lower_limits.velocity\t" << -_robot.lower_limits.velocity.get(std::get<0>(zip));
      //     std::cout << "\t _robot.upper_limits.velocity\t" << _robot.upper_limits.velocity.get(std::get<0>(zip));
      //     std::cout << "\t _robot.lower_limits.position\t" <<  (_robot.state.position.get()[std::get<0>(zip)]-_robot.lower_limits.position.get(std::get<0>(zip)))/_robot.rate();
      //     std::cout << "\t _robot.upper_limits.position\t" <<  (-_robot.state.position.get()[std::get<0>(zip)]+_robot.upper_limits.position.get(std::get<0>(zip)))/_robot.rate();
      //     std::cout << std::endl;
      //   }
      // }
      //   jacobian.topRows(_velocity_dofs.size()) = -_robot.lower_limits.velocity.get(_velocity_dofs);
      // }
      // std::cout << "state\t" << state.size() << ":\t" << state.transpose() << std::endl;

      // std::cout << "jacobian\t" << jacobian << std::endl;
    }





  protected:
    mwoibn::robot_class::Robot& _robot;
    std::vector<unsigned int>  _dofs;
    std::vector<bool> _velocity_dofs, _position_dofs;

    bool _velocity = false, _position = false;
    virtual JointConstraint* clone_impl() const override {return new JointConstraint(*this);}

};


}
}
#endif
