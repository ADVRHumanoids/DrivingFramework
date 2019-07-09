#include "mgnss/higher_level/qp/tasks/support_shaping_v5.h"



mgnss::higher_level::SupportShaping5::SupportShaping5(mwoibn::robot_class::Robot& robot, YAML::Node config,  std::vector<std::unique_ptr<mwoibn::robot_points::Rotation>>& steering_frames, const mgnss::higher_level::Limit& margin, const mgnss::higher_level::Limit& workspace, bool is_margin):
  QrTask(8+2+12, 8), _robot(robot), _margin(margin), _wheel_transforms(steering_frames), _workspace(workspace), _is_margin(is_margin){
    if(!_is_margin) resize(8,0);
}

void mgnss::higher_level::SupportShaping5::_allocate(){

          _size = 4;
          _return_state.setZero(_vars); // state + slack

          // _vector_cost_.setZero(_vars+_slack);

          soft_inequality.clear();
          hard_inequality.clear();

            // if(_is_margin)
              addSoft(Constraint(4,_vars), 5e1); // margin
              addSoft(Constraint(4,_vars), 8e1); // wirkspace

              QrTask::init();
              _cost.quadratic.setZero();
              for(int i = 0; i < 4; i++){
                // _vector_cost_[2*i] = 1;
                // _vector_cost_[2*i+1] = 3;
                _cost.quadratic(2*i, 2*i) = 1;
                _cost.quadratic(2*i+1, 2*i+1) = 1;

                // _vector_cost_[12+i] = 1000;
              }
              // _vector_cost_[8+i] = 50;

              // _cost.quadratic =_vector_cost_.asDiagonal(); // this is a velocity component
              _cost.quadratic.block(_vars, _vars, 4, 4) = soft_inequality[0].getGain().asDiagonal();
              _cost.quadratic.block(_vars+4, _vars+4, 4, 4) = soft_inequality[1].getGain().asDiagonal();
}


void mgnss::higher_level::SupportShaping5::_update(){

     _optimal_state.setZero();

    for(int i = 0; i < _size; i++){
        soft_inequality[0].setJacobian().block<4,2>(0,2*i) = _margin.getJacobian().middleCols<2>(3*i);
        // soft_inequality[1].state[4+i] = _workspace.getState()[i]/_robot.rate(); //? what is is
    //    soft_inequality[1].setState()[4+i] = -(0.0*0.0 - _workspace.getState()[i])/_robot.rate(); //Avoid going under the robot?
        soft_inequality[1].setJacobian().block<4,2>(0,2*i) = -_workspace.getJacobian().middleCols<2>(3*i);
        soft_inequality[1].setJacobian().block<4,2>(0,2*_size+2+3*i) = -_workspace.getJacobian().middleCols(3*_size+2+3*i,2);


    }
    soft_inequality[1].setState() = _workspace.error;

    //soft_inequality[1].setJacobian().block<4,8>(4,0) = _workspace.getJacobian();

      soft_inequality[0].setState() = _margin.error;
    //
    // std::cout << "soft_inequality 0\n" << soft_inequality[0].getJacobian() << std::endl;
    // std::cout << "soft_inequality 1\n" << soft_inequality[1].getJacobian() << std::endl;

    soft_inequality[0].setJacobian().block<4,2>(0,2*_size) = _margin.getJacobian().middleCols<2>(3*_size);

    QrTask::_update();
    // std::cout << "soft_inequality 0\n" << soft_inequality[0].getJacobian() << std::endl;
    // std::cout << "soft_inequality 1\n" << soft_inequality[1].getJacobian() << std::endl;
    //
    // std::cout << "cost quadratic\n" << _cost.quadratic << std::endl;
    // std::cout << "cost linear\n" << _cost.linear<< std::endl;
    // std::cout << "margin.getJacobian()\t" << soft_inequality[0].getJacobian().transpose() << std::endl;
    // std::cout << "workspace.getJacobian()\t" << soft_inequality[1].getJacobian().transpose() << std::endl;
    // std::cout << "hard_inequality\n" << hard_inequality[0].getJacobian() << std::endl;
    // std::cout << "margin.getState()\t" << soft_inequality[0].getState().transpose() << std::endl;
    // std::cout << "workspace.getState()\t" << soft_inequality[1].getState().transpose() << std::endl;
    // std::cout << "workspace.error\t" << _workspace.error.transpose() << std::endl;
    // std::cout << "margin.limit\t" << _margin.limit.transpose() << std::endl;

}

void mgnss::higher_level::SupportShaping5::log(mwoibn::common::Logger& logger){

    logger.add("cost", _optimal_cost);

    for (int i = 0; i < _vars; i++)
       logger.add("optimal_cp_" + std::to_string(i), _optimal_state[i]);

    for (int i = 0; i < _size; i++){
       //logger.add(std::string("workspace_") + std::to_string(i), std::sqrt(_workspace.getState()[i]));
       logger.add(std::string("error_workspace_") + std::to_string(i), soft_inequality[1].getState()[i]);
       logger.add(std::string("error_margin_") + std::to_string(i), soft_inequality[0].getState()[i]);
     }

    for (int i = 0; i < _slack; i++)
       logger.add(std::string("slack_") + std::to_string(i), _optimal_state[_slack+i]);

}


void mgnss::higher_level::SupportShaping5::_outputTransform(){

  // std::cout << "_cost\t" << _optimal_cost << std::endl;
  mwoibn::Vector3 temp__;
  temp__.setZero();
  // std::cout << "_steering_state\t" << _return_state.transpose() << std::endl;

  for(int i = 0; i < 4; i++){
    // temp__.head<2>() = _optimal_state.segment<2>(2*i);
    // _return_state.segment<2>(2*i) = (_wheel_transforms[i]->rotation*temp__).head<2>();
    _return_state.segment<2>(2*i) = _optimal_state.segment<2>(2*i);

  }

  // std::cout << "_world_state\t" << _return_state.transpose() << std::endl;
}
