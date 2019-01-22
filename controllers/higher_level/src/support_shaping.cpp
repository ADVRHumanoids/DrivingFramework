#include "mgnss/higher_level/support_shaping.h"




mgnss::higher_level::SupportShaping::SupportShaping(mwoibn::robot_class::Robot& robot, YAML::Node config):
  _robot(robot), _base(_robot.centerOfMass()), _contact_points(_robot.getDofs()),
   _points(_robot.getDofs()), _base_points(_robot.getDofs()), _llt(robot.getDofs()){

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

        _margins.setZero(_contact_points.size()); // How to define which margins should be computed here?
        _norms.setZero(_contact_points.size());
        _margins_jacobian.setZero(_contact_points.size(), _contact_points.size()*3+2); // points x,y, base x,y
        _margin_pairs = { {0,1}, {1,3}, {2,0}, {3,2}};
        // have contacts been updated already (?) -- not if constraints did not run yet
          for(auto& pair: _margin_pairs){
          _points.add(mwoibn::robot_points::Minus(_contact_points[pair.first], _contact_points[pair.second]));
          _base_points.add(mwoibn::robot_points::Minus(_contact_points[pair.first], _base));
        }
        // here I need to find the way to initialize the pairs
        // Minuses are intialized directly from that

        _local_cost.setZero(8,8);
        _active_links = _robot.getDof(_robot.getLinks("lower_body"));
        _optimal_state.setZero(_active_links.size());
        // std::cout << "_active_links\t" <<  _active_links.transpose() << std::endl;
        //
        // std::cout << "MAX_DOUBLE " << mwoibn::MAX_DOUBLE << std::endl;
        // std::cout << "NON_EXISTING " << mwoibn::NON_EXISTING << std::endl;
        // std::cout << "RBDL_NON_EXISTING " << mwoibn::RBDL_NON_EXISTING << std::endl;
        //

        for(int k =0, i; k < _active_links.size(); k++){
           i = _active_links[k];
          bool velocity = true, position = true;

          std::cout << mwoibn::NON_EXISTING << "\t" <<  _robot.lower_limits.velocity.get()[i] << "\t" << _robot.upper_limits.velocity.get()[i] << std::endl;

          if (_robot.lower_limits.velocity.get()[i] == mwoibn::NON_EXISTING || _robot.upper_limits.velocity.get()[i] == mwoibn::NON_EXISTING ){
            if (_robot.lower_limits.velocity.get()[i] != _robot.upper_limits.velocity.get()[i])
              throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string("Could not create support shaping task. Only one velocity limit defined for link ")
                                          + _robot.getLinks(i));
              velocity = false;
          }

          if (_robot.lower_limits.position.get()[i] == mwoibn::NON_EXISTING || _robot.upper_limits.position.get()[i] == mwoibn::NON_EXISTING ){
            if (_robot.lower_limits.position.get()[i] != _robot.upper_limits.position.get()[i])
              throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string("Could not create support shaping task. Only one position limit defined for link ")
                                          + _robot.getLinks(i));
              position = false;
          }

          _limits.push_back(std::make_tuple(i, position||velocity, position, velocity));

        }

}

void mgnss::higher_level::SupportShaping::init(){


    mwoibn::VectorN vector_cost_(8);
    vector_cost_ << 1, 1.5, 1, 1.5, 1, 1.5, 1, 1.5;
    _local_cost =vector_cost_.asDiagonal();
    _cost.setZero(_active_links.size(), _active_links.size());

    int i = 0;
    for(auto& task_: _other_tasks)
      i += task_->getTaskSize();
    _equality_matrix.setZero(_active_links.size(), i);
    _equality_vector.setZero(i);

    i = 0;
    for(auto& limits_: _limits)
      if (std::get<1>(limits_)) ++i;

    _inequality_matrix.setZero(_active_links.size(), 2*i+4 );
    _inequality_vector.setZero(2*i+4);

    for(int i; i < _limits.size(); i++ ){
      if(!std::get<1>(_limits[i])) continue;
      _inequality_matrix(std::get<0>(_limits[i]),i ) = 1;
      _inequality_matrix(std::get<0>(_limits[i]),_limits.size()+i ) = -1;
   }
}

void mgnss::higher_level::SupportShaping::update(){
    _update();
     for(int i = 0; i < _contact_points.size(); i++)
       _computeMargin(i);
     _marginJacobians();

     mwoibn::Matrix jac_(8, _active_links.size());
     mwoibn::Matrix cp_jac_(3, _active_links.size());


     for(int i = 0; i < _active_links.size(); i++)
          cp_jac_.col(i) =  _contact_points[i%3].getJacobian().block<3,1>(0,_active_links[i]);

     for(int i = 0; i < _contact_points.size(); i++)
          jac_.middleRows<2>(2*i) =  (_wheel_transforms[i]->rotation*cp_jac_).topRows<2>();

     _cost = jac_.transpose()*_local_cost*jac_;


     //_cost = _contact_points.getJacobian().transpose()*_local_cost*_contact_points.getJacobian();

     for(int i = 0; i < _active_links.size(); i++)
          _optimal_state[i] = _robot.command.velocity.get()[_active_links[i]];

     // passing this equations may be a problem
     for(int i = 0, k = 0; i < _other_tasks.size(); i++){
        for(int a = 0; a < _active_links.size(); a++)
          _equality_matrix.block(a, k, 1, _other_tasks[i]->getTaskSize()) = _other_tasks[i]->getJacobian().col(_active_links[i]).transpose();

       _equality_vector =  -_equality_matrix.transpose()*_optimal_state;
       k += _other_tasks[i]->getTaskSize();
     }

    for(int i = 0 ,j = 0; i < _limits.size(); i++ ){
      if(!std::get<1>(_limits[i])) continue;
      int k = std::get<0>(_limits[i]);
      if(std::get<2>(_limits[i]) && std::get<3>(_limits[i])){
          _inequality_vector[j] = std::min(-_robot.lower_limits.velocity.get()[k], (_robot.state.position.get()[k]-_robot.lower_limits.position.get()[k])/_robot.rate());
          _inequality_vector[(_inequality_vector.size()-4)/2+j] = std::min(_robot.upper_limits.velocity.get()[k], (_robot.upper_limits.position.get()[k]-_robot.state.position.get()[k])/_robot.rate());
      }
      else if (std::get<2>(_limits[i])){
        _inequality_vector[j] = (_robot.state.position.get()[k]-_robot.lower_limits.position.get()[k])/_robot.rate();
        _inequality_vector[(_inequality_vector.size()-4)/2+j] = (_robot.upper_limits.position.get()[k]-_robot.state.position.get()[k])/_robot.rate();
      }
      else {
        _inequality_vector[j] = -_robot.lower_limits.velocity.get()[k];
        _inequality_vector[(_inequality_vector.size()-4)/2+j] = _robot.upper_limits.velocity.get()[k];
      }

      //std::cout << _robot.getLinks(k) << "\t" << _inequality_vector[j] << "\t" << _inequality_vector[_inequality_vector.size()/2+j] << std::endl;
      j++;
   }

     jac_.setZero(14,_active_links.size());
     for(int i = 0; i < _active_links.size(); i++){
         jac_.block<2,1>(12,i) =   _robot.centerOfMass().getJacobian().block<2, 1>(0, _active_links[i]);
         jac_.block<12,1>(0,i) =  _contact_points.getJacobian().col(_active_links[i]);

    }
     //jac_.bottomRows<2>() = _robot.centerOfMass().getJacobian().topRows<2>();


    _inequality_matrix.rightCols(4) = (_margins_jacobian*jac_).transpose();
    _inequality_vector.tail(4) = (_margins - mwoibn::VectorN::Constant(4, _margin))/_robot.rate();


    // std::cout << "_inequality_matrix" << std::endl;
    // std::cout << _inequality_matrix << std::endl;
    // std::cout << "_inequality_vector" << std::endl;
    // std::cout << "_inequality_vector" << _inequality_vector.transpose() << std::endl;
     // inequality constraints
     // velocity joint limits

}

void mgnss::higher_level::SupportShaping::solve(mwoibn::common::Logger& logger){

  _llt.compute(_cost);
  _trace = _cost.trace();

  update();

  mwoibn::VectorN g0 = mwoibn::VectorN::Zero(_optimal_state.size());

  solve_quadprog2(_llt, _trace, g0, _equality_matrix, _equality_vector, _inequality_matrix, _inequality_vector, _optimal_state);


  mwoibn::VectorN state__ = _robot.state.velocity.get();
  for(int i = 0; i < _active_links.size(); i++)
   state__[_active_links[i]] = _optimal_state[i];

  mwoibn::VectorN check__ = _contact_points.getJacobian()*state__;
  for (int i = 0; i < check__.size(); i++)
     logger.add("check_cp_" + std::to_string(i), check__[i]);
}


void mgnss::higher_level::SupportShaping::_update(){
    _contact_points.update(true);
    _points.update(false);
    _base_points.update(false);
    for(auto& wheel: _wheel_transforms)
      wheel->compute();
    }

// I should check if it works before making the full optimization
void mgnss::higher_level::SupportShaping::_computeMargin(int i){
    _margins[i] = -_points[i].get()[0]*_base_points[i].get()[1]+_points[i].get()[1]*_base_points[i].get()[0];
    _norms[i] = std::sqrt( std::pow(_points[i].get()[0], 2) + std::pow(_points[i].get()[1], 2) );
    _margins[i] = _margins[i]/_norms[i];
}

// Do I have a way to validate it?
// Try to integrate and compare with next values m + dt J \dot q = m_1+i
double mgnss::higher_level::SupportShaping::_marginJacobians(){
    _margins_jacobian.setZero();
    for(int i = 0; i < _contact_points.size(); i++){
        // point 1 x
        _margins_jacobian.row(i)[_margin_pairs[i].first*3] = -_points[i].get()[0]*(_margins[i])/std::pow(_norms[i],4) - (_base_points[_margin_pairs[i].second].get()[1])/_norms[i];
        // point 1 y
        _margins_jacobian.row(i)[_margin_pairs[i].first*3+1] = -_points[i].get()[1]*(_margins[i])/std::pow(_norms[i],4) + (_base_points[_margin_pairs[i].second].get()[0])/_norms[i];
        // point 2 x
        _margins_jacobian.row(i)[_margin_pairs[i].second*3] = _points[i].get()[0]*(_margins[i])/std::pow(_norms[i],4) + (_base_points[i].get()[1])/_norms[i];
        // point 2 y
        _margins_jacobian.row(i)[_margin_pairs[i].second*3+1] = _points[i].get()[1]*(_margins[i])/std::pow(_norms[i],4) - _base_points[i].get()[0]/_norms[i];
        // base x
        _margins_jacobian.row(i)[12] = -_points[i].get()[1]/_norms[i];
        // base y
        _margins_jacobian.row(i)[13] = _points[i].get()[0]/_norms[i];
    }
}
