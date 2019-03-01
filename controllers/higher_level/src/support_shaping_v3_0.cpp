#include "mgnss/higher_level/support_shaping_v3_0.h"



mgnss::higher_level::SupportShapingV3::SupportShapingV3(mwoibn::robot_class::Robot& robot, YAML::Node config,  std::vector<std::unique_ptr<mwoibn::robot_points::Rotation>>& steering_frames, const mgnss::higher_level::Limit& margin, const mgnss::higher_level::Limit& workspace):
  _robot(robot), _llt(robot.getDofs()), _margin(margin), _wheel_transforms(steering_frames), _workspace(workspace){

          _size = 4;
          _slack = 8;
          _vars = 8;
          _optimal_state.setZero(_slack + _vars); // state + slack
          _return_state.setZero(_vars); // state + slack

        _quadratic_cost.setZero(_optimal_state.size(),_optimal_state.size());
        _linear_cost.setZero(_optimal_state.size());
        _vector_cost_.setZero(_quadratic_cost.rows());

}

void mgnss::higher_level::SupportShapingV3::init(){

    for(int i = 0; i < 4; i++){
      _vector_cost_[2*i] = 1;
      _vector_cost_[2*i+1] = 3;
      _vector_cost_[8+i] = 10;
      _vector_cost_[12+i] = 1000;
    }

    _quadratic_cost =_vector_cost_.asDiagonal(); // this is a velocity component
    _equality_matrix.setZero(0, 0);
    _equality_vector.setZero(0);

    _inequality_matrix.setZero(_optimal_state.size(), 4+4+_slack );
    _inequality_vector.setZero(4+4+_slack);
    _inequality_matrix.block(_vars, 0, _slack, 4+4 ) = -mwoibn::Matrix::Identity(_slack, 4+4);
    _inequality_matrix.block(_vars,4+4, _slack, 4+4 ) = -mwoibn::Matrix::Identity(_slack, 4+4);

}

void mgnss::higher_level::SupportShapingV3::_update(){

     _optimal_state.setZero();
     mwoibn::Matrix jac_ = mwoibn::Matrix::Zero(8,8);

   // this will be variable size set only inactive constraints - active is integrated in a cost function as a linear function add workspace limits
    for(int i = 0; i < _size; i++){
        jac_.block<4,2>(0,2*i) = _margin.jacobian.middleCols<2>(3*i);
        _inequality_vector[_size+i] = (_workspace.limit[i]*_workspace.limit[i] - _workspace.state[i])/_robot.rate();
   }
   jac_.bottomRows<4>() = -_workspace.jacobian; // minus dissapears due to the contact form


   //  for(int i = 0; i < _size; i++){
   //      jac_.block<4,2>(0,2*i) = _margin.state.jacobian.middleCols<2>(3*i);
   //      jac_.block<1,2>(_size+i,2*i) =  (2*_workspace_points[i].get()).transpose().head<2>(); // minus dissapears due to the contact form
   //      _inequality_vector[_size+i] = (_workspace[i]*_workspace[i] - _workspace_points[i].get().transpose()*_workspace_points[i].get())/_robot.rate();
   // }

    _inequality_matrix.block(0,0,8,_vars) = jac_.transpose();
    _inequality_vector.head<4>() = (_margin.state - _margin.limit)/_robot.rate();


}

void mgnss::higher_level::SupportShapingV3::log(mwoibn::common::Logger& logger){

    logger.add("cost", cost__);

    for (int i = 0; i < _vars; i++)
       logger.add("optimal_cp_" + std::to_string(i), _optimal_state[i]);

    for (int i = 0; i < _size; i++){
       logger.add(std::string("workspace_") + std::to_string(i), std::sqrt(_workspace.state[i]));
       logger.add(std::string("error_workspace_") + std::to_string(i), _inequality_vector[4+i]);
     }

    for (int i = 0; i < _slack; i++)
       logger.add(std::string("slack_") + std::to_string(i), _optimal_state[_slack+i]);

       std::cout << _optimal_state.transpose() << std::endl;

}


void mgnss::higher_level::SupportShapingV3::solve(){

  _llt.compute(_quadratic_cost);
  _trace = _quadratic_cost.trace();

  _update();

  cost__ = solve_quadprog2(_llt, _trace, _linear_cost, _equality_matrix, _equality_vector, _inequality_matrix, _inequality_vector, _optimal_state);

  std::cout << "_inequality_vector\t" << _inequality_vector.transpose() << std::endl;
  std::cout << "_inequality_matrix\n" << _inequality_matrix << std::endl;

  std::cout << "_cost\t" << cost__ << std::endl;
  mwoibn::Vector3 temp__;
  temp__.setZero();

  for(int i = 0; i < 4; i++){
    temp__.head<2>() = _optimal_state.segment<2>(2*i);
    _optimal_state.segment<2>(2*i) = (_wheel_transforms[i]->rotation*temp__).head<2>();
  }
  std::cout << "_optimal_state\t" << _optimal_state.transpose() << std::endl;

}
