#include "mgnss/higher_level/state_machine_II.h"

mgnss::higher_level::StateMachineII::StateMachineII(mwoibn::robot_class::Robot& robot, YAML::Node config):
  StateMachine(robot, config){

        int vars = _size*3+2+_size*3;
        _margins.setJacobian().setZero(_size, vars); // cp + base
        _workspace.setJacobian().setZero(_size, vars);

        state_I.offset.set().setZero(vars-_size);
        state_I.jacobian.set().setZero(vars-_size, vars);
        state_I.jacobian.set().bottomRightCorner(3*_size+2,3*_size+2).setIdentity();

        state_II.offset.set().setZero(vars);
        state_II.jacobian.set().setZero(vars, _robot.getDofs());
}


// Do I have a way to validate it?
// Try to integrate and compare with next values m + dt J \dot q = m_1+i
void mgnss::higher_level::StateMachineII::_marginJacobians(){

    _margins.setJacobian().setZero();

    for(int i = 0; i < _contact_points.size(); i++){

            // mwoibn::Vector3 point = _wheel_transforms[i]->rotation.transpose()*_points[i].get();
            // mwoibn::Vector3 base_this = _wheel_transforms[i]->rotation.transpose()*_base_points[i].get();
            // mwoibn::Vector3 base_other = _wheel_transforms[i]->rotation.transpose()*_base_points[_margin_pairs[i].second].get();
            mwoibn::Vector3 point = _points[i].get();
            mwoibn::Vector3 base_this = _base_points[i].get();
            mwoibn::Vector3 base_other = _base_points[_margin_pairs[i].second].get();

      // point 1 x
      _margins.setJacobian().row(i)[_margin_pairs[i].first*3] = -point[0]*(_margins.getState()[i])/std::pow(_norms[i],2) - (base_other[1])/_norms[i];
      // point 1 y
      _margins.setJacobian().row(i)[_margin_pairs[i].first*3+1] = -point[1]*(_margins.getState()[i])/std::pow(_norms[i],2) + (base_other[0])/_norms[i];
      // point 2 x
      _margins.setJacobian().row(i)[_margin_pairs[i].second*3] = point[0]*(_margins.getState()[i])/std::pow(_norms[i],2) + (base_this[1])/_norms[i];
      // point 2 y
      _margins.setJacobian().row(i)[_margin_pairs[i].second*3+1] = point[1]*(_margins.getState()[i])/std::pow(_norms[i],2) - base_this[0]/_norms[i];
      // base x
      _margins.setJacobian().row(i)[12] = -point[1]/_norms[i];
      // base y
      _margins.setJacobian().row(i)[13] = point[0]/_norms[i];
    }


}

void mgnss::higher_level::StateMachineII::_workspaceJacobian(){
  for(int i = 0; i < _contact_points.size(); i++){
    _workspace.setJacobian().block<1,3>(i,3*i) =  -(2*_workspace_points[i].get());
    // std::cout << "workspace.getJacobian()\t" << i << "\n" << _workspace.getJacobian().transpose() << std::endl;
    _workspace.setJacobian().block<1,3>(i,3*_size+2+3*i) =   (2*_workspace_points[i].get()); // plus?
    // std::cout << "workspace.getJacobian()\t" << i << "\n" << _workspace.getJacobian().transpose() << std::endl;
  }
  // for(int i = 0; i < _contact_points.size(); i++)
  //   _workspace.jacobian.block<8,2>(0, 2*i) = _workspace.jacobian.block<8,2>(0, 2*i)*_contact_points[i].getJacobianWheel().block<2,2>(0,0);
}

void mgnss::higher_level::StateMachineII::update(){
  StateMachine::update();
  state_I.jacobian.set().block(0,0,2*_size, 3*_size) = cost_I.jacobian.get();
  state_I.offset.set().head(2*_size) = cost_I.offset.get();

  state_II.jacobian.set().topRows(3*_size) = cost_II.jacobian.get();
  state_II.jacobian.set().row(3*_size) = _robot.centerOfMass().getJacobian().row(0);
  state_II.jacobian.set().row(3*_size+1) = _robot.centerOfMass().getJacobian().row(1);
  state_II.jacobian.set().bottomRows(3*_size) = _hips.getJacobian();
  //
  // std::cout << "state_I.jacobian\n" << state_I.jacobian.get() << std::endl;
  // std::cout << "state_I.offset\n" << state_I.offset.get().transpose() << std::endl;
  // std::cout << "state_II.jacobian\n" << state_II.jacobian.get() << std::endl;
  // std::cout << "state_II.offset\n" << state_II.offset.get().transpose() << std::endl;
}
