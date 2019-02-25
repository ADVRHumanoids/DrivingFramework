#include "mgnss/higher_level/support_shaping_v4_0.h"



mgnss::higher_level::SupportShapingV4::SupportShapingV4(mwoibn::robot_class::Robot& robot, YAML::Node config,  std::vector<std::unique_ptr<mwoibn::robot_points::Rotation>>& steering_frames, const mgnss::higher_level::Limit& margin, const mgnss::higher_level::Limit& workspace, bool is_margin):
  QrTask(8, 4), _robot(robot), _margin(margin), _wheel_transforms(steering_frames), _workspace(workspace), _is_margin(is_margin){
    if(!_is_margin) resize(8,0);
}

void mgnss::higher_level::SupportShapingV4::_allocate(){

          _size = 4;
          _return_state.setZero(_vars); // state + slack

          // _vector_cost_.setZero(_vars+_slack);

            if(_is_margin)
              soft_inequality.add(Constraint(4,_vars)); // margin
              hard_inequality.add(Constraint(8,_vars)); //limit

              QrTask::init();

              for(int i = 0; i < 4; i++){
                // _vector_cost_[2*i] = 1;
                // _vector_cost_[2*i+1] = 3;
                _cost.quadratic(2*i, 2*i) = 1;
                _cost.quadratic(2*i+1, 2*i+1) = 1;

                // _vector_cost_[12+i] = 1000;
              }
              // _vector_cost_[8+i] = 50;

              // _cost.quadratic =_vector_cost_.asDiagonal(); // this is a velocity component
              _cost.quadratic.block(_vars, _vars, _slack, _slack) = 100000*mwoibn::Matrix::Identity(_slack,_slack);

}


void mgnss::higher_level::SupportShapingV4::update(){

     _optimal_state.setZero();

    for(int i = 0; i < _size; i++){
      if(_is_margin)
        soft_inequality[0].jacobian.block<4,2>(0,2*i) = _margin.jacobian.middleCols<2>(3*i);
        hard_inequality[0].state[i] = (_workspace.limit[i]*_workspace.limit[i] - _workspace.state[i])/_robot.rate();
        hard_inequality[0].state[4+i] = _workspace.state[i]/_robot.rate();
    }

    hard_inequality[0].jacobian.block<4,8>(0,0) = -_workspace.jacobian;
    hard_inequality[0].jacobian.block<4,8>(4,0) = _workspace.jacobian;
    if(_is_margin)
      soft_inequality[0].state = (_margin.state - _margin.limit)/_robot.rate();

    QrTask::update();
    // std::cout << "soft_inequality\n" << soft_inequality[0].jacobian << std::endl;
    // std::cout << "hard_inequality\n" << hard_inequality[0].jacobian << std::endl;
    // std::cout << "workspace.state\t" << soft_inequality[1].state.transpose() << std::endl;
    // std::cout << "margin.limit\t" << _margin.limit.transpose() << std::endl;
    // std::cout << "workspace.limit\t" << hard_inequality.limit.transpose() << std::endl;

}

void mgnss::higher_level::SupportShapingV4::log(mwoibn::common::Logger& logger){

    logger.add("cost", cost__);

    for (int i = 0; i < _vars; i++)
       logger.add("optimal_cp_" + std::to_string(i), _optimal_state[i]);

    for (int i = 0; i < _size; i++){
       logger.add(std::string("workspace_") + std::to_string(i), std::sqrt(_workspace.state[i]));
       logger.add(std::string("error_workspace_") + std::to_string(i), _inequality.state[4+i]);
     }

    for (int i = 0; i < _slack; i++)
       logger.add(std::string("slack_") + std::to_string(i), _optimal_state[_slack+i]);

}


void mgnss::higher_level::SupportShapingV4::_outputTransform(){

  // std::cout << "_cost\t" << cost__ << std::endl;
  mwoibn::Vector3 temp__;
  temp__.setZero();
  // std::cout << "_steering_state\t" << _return_state.transpose() << std::endl;

  for(int i = 0; i < 4; i++){
    temp__.head<2>() = _return_state.segment<2>(2*i);
    _return_state.segment<2>(2*i) = (_wheel_transforms[i]->rotation*temp__).head<2>();
  }

  // std::cout << "_world_state\t" << _return_state.transpose() << std::endl;
}
