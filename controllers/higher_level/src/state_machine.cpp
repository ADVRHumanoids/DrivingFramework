#include "mgnss/higher_level/state_machine.h"

mgnss::higher_level::StateMachine::StateMachine(mwoibn::robot_class::Robot& robot, YAML::Node config):
  _robot(robot), _base(_robot.centerOfMass()), _contact_points(_robot.getDofs()),
   _points(_robot.getDofs()), _base_points(_robot.getDofs()),
   _hips(robot.getDofs()), _wheels(robot.getDofs()), _workspace_points(robot.getDofs()){

  // for(auto& name: _robot.getLinks("wheels"))
  //   _contact_points.add(mwoibn::robot_class::name, )
        std::vector<std::string> names = _robot.getLinks("wheels");

        for(auto& contact: _robot.contacts())
        {
            std::string name = _robot.getBodyName(contact->wrench().getBodyId());
            if(!std::count(names.begin(), names.end(), name)){
              std::cout << "Tracked point " << name << " could not be initialized" << std::endl;
              names.erase(std::remove(names.begin(), names.end(), name), names.end());
              continue;
            }


            std::unique_ptr<mwoibn::robot_points::TorusModel> torus_(new mwoibn::robot_points::TorusModel(
                               _robot, mwoibn::point_handling::FramePlus(name,
                               _robot.getModel(), _robot.state),
                               mwoibn::Axis(config["reference_axis"][name]["x"].as<double>(),
                                            config["reference_axis"][name]["y"].as<double>(),
                                            config["reference_axis"][name]["z"].as<double>()),
                                            config["minor_axis"].as<double>(), config["major_axis"].as<double>(),
                                            contact->getGroundNormal()));

            _wheel_transforms.push_back(std::unique_ptr<mwoibn::robot_points::Rotation>(
                      new mwoibn::robot_points::GroundWheel(torus_->axis(), torus_->groundNormal())));
            _contact_points.add(std::move(torus_));
            std::cout << "contacts: " << contact->getName() << "\t" << name << std::endl;
        }

        _margins.state.setZero(_contact_points.size()); // How to define which margins should be computed here?
        _norms.setZero(_contact_points.size());
        _margins.jacobian.setZero(_contact_points.size(), _contact_points.size()*3+2); // points x,y, base x,y
        _margin_pairs = { {0,1}, {1,3}, {2,0}, {3,2}};

          for(auto& pair: _margin_pairs){
          _points.add(mwoibn::robot_points::Minus(_contact_points[pair.first], _contact_points[pair.second]));
          _base_points.add(mwoibn::robot_points::Minus(_contact_points[pair.first], _base));
        }
        // here I need to find the way to initialize the pairs
        // Minuses are intialized directly from that

        for(auto name: _robot.getLinks("hips"))
          _hips.add(mwoibn::robot_points::LinearPoint(name, _robot));

        for(auto name: _robot.getLinks("wheels"))
          _wheels.add(mwoibn::robot_points::LinearPoint(name, _robot));

        for(int i = 0; i < _wheels.size(); i++)
          _workspace_points.add(mwoibn::robot_points::Minus(_hips[i], _wheels[i]));

        _size = 4;

        _margins.limit.setZero(4);
        _workspace.limit.setZero(4);
        _margins.limit << 0.18, 0.18, 0.18, 0.18;
        _workspace.limit << 0.70, 0.70, 0.70, 0.70; // 0.70
        _margins.state.setZero(_size);
        _workspace.state.setZero(_size);
        _margins.jacobian.setZero(_size, _size*3+2); // cp + base
        _workspace.jacobian.setZero(_size, _size*2);
}

void mgnss::higher_level::StateMachine::init(){
_counter = 25; }



// I should check if it works before making the full optimization
void mgnss::higher_level::StateMachine::_computeMargin(int i){
    _margins.state[i] = -_points[i].get()[0]*_base_points[i].get()[1]+_points[i].get()[1]*_base_points[i].get()[0];
    _norms[i] = std::sqrt( std::pow(_points[i].get()[0], 2) + std::pow(_points[i].get()[1], 2) );
    _margins.state[i] = _margins.state[i]/_norms[i];
}

// Do I have a way to validate it?
// Try to integrate and compare with next values m + dt J \dot q = m_1+i
void mgnss::higher_level::StateMachine::_marginJacobians(){

    _margins.jacobian.setZero();

    for(int i = 0; i < _contact_points.size(); i++){

            mwoibn::Vector3 point = _wheel_transforms[i]->rotation.transpose()*_points[i].get();
            mwoibn::Vector3 base_this = _wheel_transforms[i]->rotation.transpose()*_base_points[i].get();
            mwoibn::Vector3 base_other = _wheel_transforms[i]->rotation.transpose()*_base_points[_margin_pairs[i].second].get();

        // // point 1 x
        // _margins.jacobian.row(i)[_margin_pairs[i].first*3] = -_points[i].get()[0]*(_margins[i])/std::pow(_norms[i],2) - (_base_points[_margin_pairs[i].second].get()[1])/_norms[i];
        // // point 1 y
        // _margins.jacobian.row(i)[_margin_pairs[i].first*3+1] = -_points[i].get()[1]*(_margins[i])/std::pow(_norms[i],2) + (_base_points[_margin_pairs[i].second].get()[0])/_norms[i];
        // // point 2 x
        // _margins.jacobian.row(i)[_margin_pairs[i].second*3] = _points[i].get()[0]*(_margins[i])/std::pow(_norms[i],2) + (_base_points[i].get()[1])/_norms[i];
        // // point 2 y
        // _margins.jacobian.row(i)[_margin_pairs[i].second*3+1] = _points[i].get()[1]*(_margins[i])/std::pow(_norms[i],2) - _base_points[i].get()[0]/_norms[i];
        // // base x
        // _margins.jacobian.row(i)[12] = -_points[i].get()[1]/_norms[i];
        // // base y
        // _margins.jacobian.row(i)[13] = _points[i].get()[0]/_norms[i];

      // point 1 x
      _margins.jacobian.row(i)[_margin_pairs[i].first*3] = -point[0]*(_margins.state[i])/std::pow(_norms[i],2) - (base_other[1])/_norms[i];
      // point 1 y
      _margins.jacobian.row(i)[_margin_pairs[i].first*3+1] = -point[1]*(_margins.state[i])/std::pow(_norms[i],2) + (base_other[0])/_norms[i];
      // point 2 x
      _margins.jacobian.row(i)[_margin_pairs[i].second*3] = point[0]*(_margins.state[i])/std::pow(_norms[i],2) + (base_this[1])/_norms[i];
      // point 2 y
      _margins.jacobian.row(i)[_margin_pairs[i].second*3+1] = point[1]*(_margins.state[i])/std::pow(_norms[i],2) - base_this[0]/_norms[i];
      // base x
      _margins.jacobian.row(i)[12] = -point[1]/_norms[i];
      // base y
      _margins.jacobian.row(i)[13] = point[0]/_norms[i];
    }

}

void mgnss::higher_level::StateMachine::_computeWorkspace(){
  for(int i = 0; i < _contact_points.size(); i++)
    _workspace.state[i] = _workspace_points[i].get().transpose()*_workspace_points[i].get();
}

void mgnss::higher_level::StateMachine::_workspaceJacobian(){
  for(int i = 0; i < _contact_points.size(); i++)
    _workspace.jacobian.block<1,2>(i,2*i) =  -(2*_wheel_transforms[i]->rotation.transpose()*_workspace_points[i].get()).head<2>();
}


void mgnss::higher_level::StateMachine::_update(){
    _contact_points.update(true);
    _points.update(false);
    _base_points.update(false);

    for(auto& wheel: _wheel_transforms)
      wheel->compute();

    _wheels.update(false);
    _hips.update(false);
    _workspace_points.update(false);


}
