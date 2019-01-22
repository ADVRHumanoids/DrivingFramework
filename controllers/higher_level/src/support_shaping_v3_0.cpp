#include "mgnss/higher_level/support_shaping_v3_0.h"



mgnss::higher_level::SupportShapingV3::SupportShapingV3(mwoibn::robot_class::Robot& robot, YAML::Node config):
  _robot(robot), _base(_robot.centerOfMass()), _contact_points(_robot.getDofs()),
   _points(_robot.getDofs()), _base_points(_robot.getDofs()), _llt(robot.getDofs()),
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

        _margins.setZero(_contact_points.size()); // How to define which margins should be computed here?
        _norms.setZero(_contact_points.size());
        _margins_jacobian.setZero(_contact_points.size(), _contact_points.size()*3+2); // points x,y, base x,y
        _margin_pairs = { {0,1}, {1,3}, {2,0}, {3,2}};

          for(auto& pair: _margin_pairs){
          _points.add(mwoibn::robot_points::Minus(_contact_points[pair.first], _contact_points[pair.second]));
          _base_points.add(mwoibn::robot_points::Minus(_contact_points[pair.first], _base));
        }
        // here I need to find the way to initialize the pairs
        // Minuses are intialized directly from that

        for(auto name: _robot.getLinks("hips")){
          std::cout << name << std::endl;
          _hips.add(mwoibn::robot_points::LinearPoint(name, _robot));
        }
        for(auto name: _robot.getLinks("wheels")){
          std::cout << name << std::endl;
          _wheels.add(mwoibn::robot_points::LinearPoint(name, _robot));
        }
        for(int i = 0; i < _wheels.size(); i++)
          _workspace_points.add(mwoibn::robot_points::Minus(_hips[i], _wheels[i]));

          _slack = 8;
          _vars = 8;
          _optimal_state.setZero(_slack + _vars); // state + slack
          _return_state.setZero(_vars); // state + slack

        _quadratic_cost.setZero(_optimal_state.size(),_optimal_state.size());
        _linear_cost.setZero(_optimal_state.size());
        _vector_cost_.setZero(_quadratic_cost.rows());

        _safety.setZero(4);
        _workspace.setZero(4);
        _safety << 0.40, 0.20, 0.20, 0.40;
        _workspace << 0.60, 0.60, 0.60, 0.60; // 0.70
}

void mgnss::higher_level::SupportShapingV3::init(){

    for(int i = 0; i < 4; i++){
      _vector_cost_[2*i] = 1;
      _vector_cost_[2*i+1] = 3;
      _vector_cost_[8+i] = 1;
      _vector_cost_[12+i] = 0.5;
    }

    _quadratic_cost =_vector_cost_.asDiagonal(); // this is a velocity component
    _equality_matrix.setZero(0, 0);
    _equality_vector.setZero(0);

    _inequality_matrix.setZero(_optimal_state.size(), 4+4+_slack );
    _inequality_vector.setZero(4+4+_slack);
    _inequality_matrix.block(0,_vars, _slack, _vars ) = -mwoibn::Matrix::Identity(8,8);
    _inequality_matrix.block(4+4,_vars, _slack, _vars ) = mwoibn::Matrix::Identity(8,8);

}

void mgnss::higher_level::SupportShapingV3::update(){
    _update();

     for(int i = 0; i < _contact_points.size(); i++)
       _computeMargin(i);
     _marginJacobians();
     _optimal_state.setZero();
     mwoibn::Matrix jac_ = mwoibn::Matrix::Zero(8,_optimal_state.size());

   // this will be variable size set only inactive constraints - active is integrated in a cost function as a linear function add workspace limits
    for(int i = 0; i < _contact_points.size(); i++){
        jac_.block<4,2>(0,2*i) = _margins_jacobian.middleCols<2>(3*i);
        jac_.block<1,2>(_contact_points.size()+i,2*i) =  (2*_wheel_transforms[i]->rotation.transpose()*_workspace_points[i].get()).head<2>(); // minus dissapears due to the contact form
        _inequality_vector[_contact_points.size()+i] = (_workspace[i]*_workspace[i] - _workspace_points[i].get().transpose()*_workspace_points[i].get())/_robot.rate();
   }


   //  for(int i = 0; i < _contact_points.size(); i++){
   //      jac_.block<4,2>(0,2*i) = _margins_jacobian.middleCols<2>(3*i);
   //      jac_.block<1,2>(_contact_points.size()+i,2*i) =  (2*_workspace_points[i].get()).transpose().head<2>(); // minus dissapears due to the contact form
   //      _inequality_vector[_contact_points.size()+i] = (_workspace[i]*_workspace[i] - _workspace_points[i].get().transpose()*_workspace_points[i].get())/_robot.rate();
   // }

    _inequality_matrix.block(0,0,8,_vars) = jac_.transpose();
    _inequality_vector.head<4>() = (_margins - _safety)/_robot.rate();

    // std::cout << "_margins\t" << _margins.transpose() <<  std::endl;
    // std::cout << "matrix\t" << _inequality_matrix << std::endl;
    // std::cout << "vector\t" << _inequality_vector.transpose() << std::endl;
}

void mgnss::higher_level::SupportShapingV3::log(mwoibn::common::Logger& logger){

    logger.add("cost", cost__);

    for (int i = 0; i < _vars; i++)
       logger.add("optimal_cp_" + std::to_string(i), _optimal_state[i]);

    for (int i = 0; i < _contact_points.size(); i++){
       logger.add(std::string("workspace_") + std::to_string(i), _workspace_points[i].get().transpose()*_workspace_points[i].get());
       logger.add(std::string("error_workspace_") + std::to_string(i), _inequality_vector[4+i]);
     }

    for (int i = 0; i < _slack; i++)
       logger.add(std::string("slack_") + std::to_string(i), _optimal_state[_slack+i]);

}


void mgnss::higher_level::SupportShapingV3::solve(){

  _llt.compute(_quadratic_cost);
  _trace = _quadratic_cost.trace();

  update();


  cost__ = solve_quadprog2(_llt, _trace, _linear_cost, _equality_matrix, _equality_vector, _inequality_matrix, _inequality_vector, _optimal_state);

  mwoibn::Vector3 temp__;
  temp__.setZero();

  for(int i = 0; i < 4; i++){
    temp__.head<2>() = _optimal_state.segment<2>(2*i);
    _optimal_state.segment<2>(2*i) = (_wheel_transforms[i]->rotation*temp__).head<2>();
  }

}


void mgnss::higher_level::SupportShapingV3::_update(){
    _contact_points.update(true);
    _points.update(false);
    _base_points.update(false);

    for(auto& wheel: _wheel_transforms)
      wheel->compute();

    //   std::cout << "world\t" << _points.getState().transpose() << std::endl;
    // for(int i= 0; i < _points.size(); i++){
    //   _points[i].rotateFrom(*_wheel_transforms[i]);
    //   _base_points[i].rotateFrom(*_wheel_transforms[i]);
    // }
    // std::cout << "steering\t" << _points.getState().transpose() << std::endl;


    _wheels.update(false);
    _hips.update(false);
    _workspace_points.update(false);
}

// I should check if it works before making the full optimization
void mgnss::higher_level::SupportShapingV3::_computeMargin(int i){
    _margins[i] = -_points[i].get()[0]*_base_points[i].get()[1]+_points[i].get()[1]*_base_points[i].get()[0];
    _norms[i] = std::sqrt( std::pow(_points[i].get()[0], 2) + std::pow(_points[i].get()[1], 2) );
    _margins[i] = _margins[i]/_norms[i];
}

// Do I have a way to validate it?
// Try to integrate and compare with next values m + dt J \dot q = m_1+i
double mgnss::higher_level::SupportShapingV3::_marginJacobians(){
    _margins_jacobian.setZero();


    for(int i = 0; i < _contact_points.size(); i++){

      mwoibn::Vector3 point = _wheel_transforms[i]->rotation.transpose()*_points[i].get();
      mwoibn::Vector3 base_this = _wheel_transforms[i]->rotation.transpose()*_points[i].get();
      mwoibn::Vector3 base_other = _wheel_transforms[i]->rotation.transpose()*_points[i].get();

        // // point 1 x
        // _margins_jacobian.row(i)[_margin_pairs[i].first*3] = -_points[i].get()[0]*(_margins[i])/std::pow(_norms[i],2) - (_base_points[_margin_pairs[i].second].get()[1])/_norms[i];
        // // point 1 y
        // _margins_jacobian.row(i)[_margin_pairs[i].first*3+1] = -_points[i].get()[1]*(_margins[i])/std::pow(_norms[i],2) + (_base_points[_margin_pairs[i].second].get()[0])/_norms[i];
        // // point 2 x
        // _margins_jacobian.row(i)[_margin_pairs[i].second*3] = _points[i].get()[0]*(_margins[i])/std::pow(_norms[i],2) + (_base_points[i].get()[1])/_norms[i];
        // // point 2 y
        // _margins_jacobian.row(i)[_margin_pairs[i].second*3+1] = _points[i].get()[1]*(_margins[i])/std::pow(_norms[i],2) - _base_points[i].get()[0]/_norms[i];
        // // base x
        // _margins_jacobian.row(i)[12] = -_points[i].get()[1]/_norms[i];
        // // base y
        // _margins_jacobian.row(i)[13] = _points[i].get()[0]/_norms[i];

      // point 1 x
      _margins_jacobian.row(i)[_margin_pairs[i].first*3] = -point[0]*(_margins[i])/std::pow(_norms[i],2) - (base_other[1])/_norms[i];
      // point 1 y
      _margins_jacobian.row(i)[_margin_pairs[i].first*3+1] = -point[1]*(_margins[i])/std::pow(_norms[i],2) + (base_other[0])/_norms[i];
      // point 2 x
      _margins_jacobian.row(i)[_margin_pairs[i].second*3] = point[0]*(_margins[i])/std::pow(_norms[i],2) + (base_this[1])/_norms[i];
      // point 2 y
      _margins_jacobian.row(i)[_margin_pairs[i].second*3+1] = point[1]*(_margins[i])/std::pow(_norms[i],2) - base_this[0]/_norms[i];
      // base x
      _margins_jacobian.row(i)[12] = -point[1]/_norms[i];
      // base y
      _margins_jacobian.row(i)[13] = point[0]/_norms[i];
    }

}
