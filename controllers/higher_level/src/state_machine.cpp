#include "mgnss/higher_level/state_machine.h"

mgnss::higher_level::StateMachine::StateMachine(mwoibn::robot_class::Robot& robot, YAML::Node config):
  _robot(robot), _base(_robot.centerOfPressure()), _contact_points(_robot.getDofs()),
   _points(_robot.getDofs()), _base_points(_robot.getDofs()),
   _hips(robot.getDofs()), _wheels(robot.getDofs()), _workspace_points(robot.getDofs()), cost_I(robot){

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
                               _robot.getModel(), _robot.state, mwoibn::point_handling::FramePlus(name,
                               _robot.getModel(), _robot.state),
                               mwoibn::Axis(config["reference_axis"][name]["x"].as<double>(),
                                            config["reference_axis"][name]["y"].as<double>(),
                                            config["reference_axis"][name]["z"].as<double>()),
                                            config["minor_axis"].as<double>(), config["major_axis"].as<double>(),
                                            contact->getGroundNormal()));

            _wheel_transforms.push_back(std::unique_ptr<mwoibn::robot_points::Rotation>(
                      new mwoibn::robot_points::GroundWheel(torus_->axis(), torus_->groundNormal())));

            _contact_points.add(std::move(torus_));
            cost_I.torus_acceleration.add(mwoibn::dynamic_points::TorusIntegratedRoll(_contact_points.end(0), _robot ) );
            std::cout << "contacts: " << contact->getName() << "\t" << name << std::endl;
        }

        _margins.setState().setZero(_contact_points.size()); // How to define which margins should be computed here?
        _norms.setZero(_contact_points.size());
        _margins.setJacobian().setZero(_contact_points.size(), _contact_points.size()*3+2); // points x,y, base x,y
        _margin_pairs = { {0,1}, {1,3}, {2,0}, {3,2}};

          for(auto& pair: _margin_pairs){
          _points.add(mwoibn::robot_points::Minus(_contact_points[pair.first], _contact_points[pair.second]));
          _base_points.add(mwoibn::robot_points::Minus(_contact_points[pair.first], _base));
        }
        // here I need to find the way to initialize the pairs
        // Minuses are intialized directly from that

        for(auto name: _robot.getLinks("hips"))
          _hips.add(mwoibn::robot_points::LinearPoint(name, _robot));

        for(auto name: _robot.getLinks("wheels")){
          _wheels.add(mwoibn::robot_points::LinearPoint(name, _robot));
          _wheel_orientation.add(mwoibn::robot_points::FrameOrientation(name, _robot));
        }

        for(int i = 0; i < _wheels.size(); i++)
          _workspace_points.add(mwoibn::robot_points::Minus(_hips[i], _wheels[i]));

        _size = _contact_points.size();

        _margins.limit.setZero(_size);
        _workspace.limit.setZero(_size);
        //_margins.limit << 0.40, 0.18, 0.18, 0.40;
        //_margins.limit << 0.40, 0.10, 0.10, 0.40;

        // RANGES_FOR?
        YAML::Node config_ = mwoibn::robot_class::Robot::checkEntry(config, "margins", "StateMachine");
        std::cout << "State Machine read safety margins ";
        for (auto&& [id, entry] : ranges::view::enumerate(config_)){
          _margins.limit[id] = entry.as<double>();
          std::cout << entry << "\t";
        }
        std::cout << std::endl;

        // std::cout << "State Machine read workspace limits ";
        // for (auto&& [entry, id] : ranges::view::zip(config["workspaces"], ranges::view::indices)){
        //   _workspace.limit[id] = entry.as<double>();
        //   std::cout << entry << "\t";
        // }
        // std::cout << std::endl;
        config_ = mwoibn::robot_class::Robot::checkEntry(config, "workspaces", "StateMachine");
        std::cout << "State Machine read workspace limits ";
        for (auto&& [id, entry] :  ranges::view::enumerate(config_)){
          _workspace.limit[id] = entry.as<double>();
          std::cout << entry << "\t";
        }
        std::cout << std::endl;

        // _margins.limit << 0.40, 0.215, 0.215, 0.40;
        // _workspace.limit << 0.65, 0.65, 0.65, 0.65; // 0.70
        _margins.setState().setZero(_size);
        _workspace.setState().setZero(_size);
        _margins.setJacobian().setZero(_size, _size*3+2); // cp + base
        _workspace.setJacobian().setZero(_size, _size*2);
        cost_I.offset.set().setZero(_contact_points.size()*2);
        cost_I.jacobian.set().setZero(_contact_points.size()*2, 3*_size);

        cost_II.offset.set().setZero(_contact_points.size()*3);
        cost_II.jacobian.set().setZero(_contact_points.size()*3, _robot.getDofs());


}

void mgnss::higher_level::StateMachine::init(){ }



// I should check if it works before making the full optimization
void mgnss::higher_level::StateMachine::_computeMargin(int i){
    _margins.setState()[i] = -_points[i].get()[0]*_base_points[i].get()[1]+_points[i].get()[1]*_base_points[i].get()[0];
    _norms[i] = std::sqrt( std::pow(_points[i].get()[0], 2) + std::pow(_points[i].get()[1], 2) );
    _margins.setState()[i] = _margins.getState()[i]/_norms[i];
}

// Do I have a way to validate it?
// Try to integrate and compare with next values m + dt J \dot q = m_1+i
void mgnss::higher_level::StateMachine::_marginJacobians(){

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

void mgnss::higher_level::StateMachine::update(){
  _update();

  // ACCELERATION BASED
  cost_I.update();
  cost_II.jacobian.set() = _wheel_orientation.jacobian();

  // VELOCITY BASED
  // for(int i = 0; i < _contact_points.size(); i++)
  //   // temp__.setZero();
  //   _state_jacobian.middleRows<2>(2*i) = (_wheel_transforms[i]->rotation.transpose()*_contact_points[i].getJacobian()).topRows<2>();



  for(int i = 0; i < _size; i++)
     _computeMargin(i);

  _marginJacobians();
  _computeWorkspace();
  _workspaceJacobian();

  _margins.error = (_margins.getState() - _margins.limit)/_robot.rate();
  _workspace.error = (_workspace.limit.cwiseProduct(_workspace.limit) - _workspace.getState())/_robot.rate();

  //std::cout << "_margins\t" << _margins.error.transpose() << std::endl;
  //std::cout << "_workspace\t" << _workspace.error.transpose() << std::endl;
}

void mgnss::higher_level::StateMachine::_computeWorkspace(){
  for(int i = 0; i < _contact_points.size(); i++)//{
    _workspace.setState()[i] = _workspace_points[i].get().transpose()*_workspace_points[i].get();

 // }
}

void mgnss::higher_level::StateMachine::_workspaceJacobian(){
  for(int i = 0; i < _contact_points.size(); i++)
    _workspace.setJacobian().block<1,2>(i,2*i) =  -(2*_workspace_points[i].get()).head<2>();
  // for(int i = 0; i < _contact_points.size(); i++)
  //   _workspace.jacobian.block<8,2>(0, 2*i) = _workspace.jacobian.block<8,2>(0, 2*i)*_contact_points[i].getJacobianWheel().block<2,2>(0,0);
}


void mgnss::higher_level::StateMachine::_update(){
    _robot.centerOfPressure().update();
    _contact_points.update(true);
    _points.update(false);
    _base_points.update(true);// false

    for(auto& wheel: _wheel_transforms)
      wheel->compute();

    _wheels.update(false);
    _wheel_orientation.update(true);
    _wheel_orientation.getJacobian();

    _hips.update(true);
    _workspace_points.update(false);


}

void mgnss::higher_level::StateMachine::log(mwoibn::common::Logger& logger){

  for(int k = 0; k < 4; k++){
    // for(int i = 0; i < 3; i++){
    //   logger.add(std::string("est_") + std::to_string(k) + std::string("_") + char('x'+i), cost_i.torus_acceleration[k].getEstimate()[i]);
    //   logger.add(std::string("vel_") + std::to_string(k) + std::string("_")+ char('x'+i), cost_i.torus_acceleration[k].getVelocity()[i]);
    // }

    logger.add(std::string("mar_") + std::to_string(k), _margins.getState()[k]);
    logger.add(std::string("work_") + std::to_string(k), _workspace.getState()[k]);

  }

}


void mgnss::higher_level::SupportState::update(){
  torus_acceleration.update(true);


  for(int i = 0; i < torus_acceleration.size(); i++){
    //  mwoibn::Matrix3 toN, toPN;
    //  toN.noalias() = torus_acceleration[i].torus().groundNormal()*torus_acceleration[i].torus().groundNormal().transpose();
    //  toPN = mwoibn::Matrix3::Identity() - toN;
    //
    // // VELOCITY
    // _support_jacobian = torus_acceleration[i].torus().getJacobianWheel()/_robot.rate();
    //
    // mat_1.noalias() = torus_acceleration[i].getDependant()*toPN;
    // mat_1 -= _support_jacobian;
    // _support_offset.noalias() =  mat_1*torus_acceleration[i].torus().wheelAngularVelocity();
    //
    // // _support_offset =    (torus_acceleration[i].getDependant()*toPN -_support_jacobian )*torus_acceleration[i].torus().wheelAngularVelocity();
    // _support_offset += torus_acceleration[i].getIndependant();
    // _support_jacobian.noalias() += torus_acceleration[i].getDependant()*toN;
    // mat_1 = _support_jacobian*_robot.rate();
    // vec_1 = _support_offset*_robot.rate();
    //
    // vec_1.noalias() += torus_acceleration[i].torus().getJacobian()*_robot.state.velocity.get();
    //
    // // std::cout << "_state_machine\n" << _support_jacobian  << std::endl;
    // // _support_jacobian.noalias() = _wheel_transforms[i]->rotation.transpose()*mat_1;
    // // _support_offset.noalias() = _wheel_transforms[i]->rotation.transpose()*vec_1;
    // _support_jacobian.noalias() = mat_1;
    // _support_offset.noalias() = vec_1;

    jacobian.set().block<2,3>(2*i, 3*i) = torus_acceleration[i].getJacobianWheel().topRows<2>();
    // offset.set().segment<2>(2*i) = 0.5*torus_acceleration[i].getConstant().head<2>();  // ? minus

  }

}
