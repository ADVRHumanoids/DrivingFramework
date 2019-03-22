#include "mgnss/higher_level/qp/tasks/qr_tracking.h"



mgnss::higher_level::QrTracking::QrTracking(mwoibn::robot_class::Robot& robot, YAML::Node config, const mwoibn::VectorN& reference, const mwoibn::VectorN& current, double gain, std::vector<std::unique_ptr<mwoibn::robot_points::Rotation>>& steering_frames, const mgnss::higher_level::Limit& margin, const mgnss::higher_level::Limit& workspace):
  _robot(robot), _reference(reference), _current(current), _llt(robot.getDofs()), _wheel_transforms(steering_frames), _margin(margin), _workspace(workspace){

        // for(auto name: _robot.getLinks("hips"))
        //   _hips.add(mwoibn::robot_points::LinearPoint(name, _robot));
        //
        // for(auto name: _robot.getLinks("wheels"))
        //   _wheels.add(mwoibn::robot_points::LinearPoint(name, _robot));
        //
        // for(int i = 0; i < _wheels.size(); i++)
        //   _workspace_points.add(mwoibn::robot_points::Minus(_hips[i], _wheels[i]));

          _slack = 8;
          _vars = 8;
          _optimal_state.setZero(_slack + _vars); // state + slack
          _return_state.setZero(_vars); // state + slack

        _quadratic_cost.setZero(_optimal_state.size(),_optimal_state.size());
        _linear_cost.setZero(_optimal_state.size());
        _vector_cost_.setZero(_quadratic_cost.rows());

        _desired.setZero(_vars);
        // _margin.limit.setZero(4);
        // // _workspace.limit.setZero(4);
        // _margin.limit << 0.20, 0.20, 0.20, 0.20;
        // _workspace.limit << 0.70, 0.70, 0.70, 0.70; // 0.70
}

void mgnss::higher_level::QrTracking::init(){

    for(int i = 0; i < 4; i++){
      _vector_cost_[2*i] = 1;
      _vector_cost_[2*i+1] = 1;
      _vector_cost_[8+i] = 100000;
      _vector_cost_[12+i] = 100000;
    }

    _quadratic_cost =_vector_cost_.asDiagonal(); // this is a velocity component
    _equality_matrix.setZero(0, 0);
    _equality_vector.setZero(0);

    _inequality_matrix.setZero(_optimal_state.size(), 4+4+_slack + _vars*2 );
    _inequality_vector.setZero(4+4+_slack+2*_vars);
    _inequality_matrix.block(_vars, 0, _slack, 4+4 ) = -mwoibn::Matrix::Identity(_slack, 4+4);
    _inequality_matrix.block(_vars,4+4, _slack, 4+4 ) = -mwoibn::Matrix::Identity(_slack, 4+4);
    _inequality_matrix.block(0,4+4+_slack, _vars, _vars ) = -mwoibn::Matrix::Identity(_vars, _vars); // max
    _inequality_matrix.block(0,4+4+_slack+_vars, _vars, _vars ) = mwoibn::Matrix::Identity(_vars, _vars); // min

    _offset_margin.setZero(4);
    _offset_workspace.setZero(4);
    _offset_margin << _margin.limit.array().sign()*0.002;
    _offset_workspace << _workspace.limit.array().sign()*0.002;
    std::cout << "_offset_margin\t" << _inequality_matrix.transpose() << std::endl;
    std::cout << "_offset_margin\t" << _inequality_vector.transpose() << std::endl;

    std::cout << "_offset_margin\t" << _offset_margin.transpose() << std::endl;
    std::cout << "_offset_workspace\t" << _offset_workspace.transpose() << std::endl;
}

void mgnss::higher_level::QrTracking::_update(){
    // _update();

      // I have to consider a robot heading here?


     for(int i = 0; i < _size; i++)
       _desired.segment<2>(2*i) = (_wheel_transforms[i]->rotation.transpose()*(_reference - _current).segment<3>(3*i)/_robot.rate()).head<2>();


     std::cout << "_desired\t" << _desired.transpose() << std::endl;
     std::cout << "_reference\t" << _reference.transpose() << std::endl;
     std::cout << "_current\t" << _current.transpose() << std::endl;

     // _marginJacobians();
     _optimal_state.setZero();
     mwoibn::Matrix jac_ = mwoibn::Matrix::Zero(8,_vars);

   // this will be variable size set only inactive constraints - active is integrated in a cost function as a linear function add workspace limits
    for(int i = 0; i < _size; i++){
        jac_.block<4,2>(0,2*i) = _margin.getJacobian().middleCols<2>(3*i);
        _inequality_vector[_size+i] = ((_workspace.limit[i]-_offset_workspace[i])*(_workspace.limit[i]-_offset_workspace[i]) - _workspace.getState()[i])/_robot.rate();
   }
   jac_.bottomRows<4>() = -_workspace.getJacobian(); // minus dissapears due to the contact form


    _inequality_matrix.block(0,0,8,_vars) = jac_.transpose();
    _inequality_vector.head<4>() = (_margin.getState() - (_margin.limit + _offset_margin))/_robot.rate();

    // std::cout << "vector\t" << _inequality_vector.transpose() << std::endl;

    _inequality_vector.head(_vars) += _inequality_matrix.block(0,0,8,_vars).transpose()*_desired;
    // std::cout << "_margin.state\t" << _margin.state.transpose() <<  std::endl;
    _inequality_vector.segment(4+4+_slack , _vars).setConstant(1.5);
    _inequality_vector.segment(4+4+_slack , _vars) -= _desired;

    _inequality_vector.segment(4+4+_slack+_vars , _vars).setConstant(1.5);
    _inequality_vector.segment(4+4+_slack+_vars , _vars) += _desired;

    // std::cout << "jac\t" << jac_ << std::endl;

    // std::cout << "matrix\t" << _inequality_matrix << std::endl;
    std::cout << "vector\t" << _inequality_vector.transpose() << std::endl;
}

void mgnss::higher_level::QrTracking::log(mwoibn::common::Logger& logger){

    logger.add("cost", _optimal_cost);

    for (int i = 0; i < _vars; i++){
       logger.add("optimal_cp_" + std::to_string(i), _optimal_state[i]);
       logger.add("final_cp_" + std::to_string(i), _return_state[i]);
     }

    for (int i = 0; i < _size; i++){
       logger.add(std::string("workspace_") + std::to_string(i), std::sqrt(_workspace.getState()[i]));
       logger.add(std::string("error_workspace_") + std::to_string(i), _inequality_vector[4+i]);
     }

    for (int i = 0; i < _slack; i++)
       logger.add(std::string("slack_") + std::to_string(i), _optimal_state[_slack+i]);

       // std::cout << "margins\t" << _margin.state.transpose() << std::endl;

}


void mgnss::higher_level::QrTracking::solve(){

  _llt.compute(_quadratic_cost);
  _trace = _quadratic_cost.trace();

  _update();


  _optimal_cost = solve_quadprog2(_llt, _trace, _linear_cost, _equality_matrix, _equality_vector, _inequality_matrix, _inequality_vector, _optimal_state);

  //
  //std::cout << std::setprecision(16) << "_optimal_cost\t" << _optimal_cost << std::endl;

  mwoibn::Vector3 temp__;
  temp__.setZero();

  // std::cout << "_optimal_state\t" << _optimal_state.transpose() <<  std::endl;
  // std::cout << "_optimal_cost\t" << _optimal_cost <<  std::endl;

  _optimal_state.head(_vars) += _desired;

  for(int i = 0; i < 4; i++){
    temp__.head<2>() = _optimal_state.segment<2>(2*i);
    _optimal_state.segment<2>(2*i) = (_wheel_transforms[i]->rotation*temp__).head<2>();
  }

  _return_state = _optimal_state.head(_vars);
  std::cout << "_result_state\t" << _optimal_state.transpose() <<  std::endl;

}

//
// void mgnss::higher_level::QrTracking::_update(){
//     _wheels.update(false);
//     _hips.update(false);
//     //_workspace_points.update(false);
// }
