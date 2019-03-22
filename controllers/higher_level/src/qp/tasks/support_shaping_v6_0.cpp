#include "mgnss/higher_level/qp/tasks/support_shaping_v6_0.h"



mgnss::higher_level::SupportShapingV6::SupportShapingV6(mwoibn::robot_class::Robot& robot, YAML::Node config,  std::vector<std::unique_ptr<mwoibn::robot_points::Rotation>>& steering_frames, const mgnss::higher_level::Limit& margin, const mgnss::higher_level::Limit& workspace, mwoibn::robot_points::Handler<mwoibn::dynamic_points::Torus>& accelerations):
  QrTask(12, 4), _original_task(robot, config, steering_frames, margin, workspace), _accelerations(accelerations), _robot(robot){

}

void mgnss::higher_level::SupportShapingV6::_allocate(){

  // _size = 4;
  _return_state.setZero(_vars); // state + slack

  // _vector_cost_.setZero(_vars+_slack);

    // if(_is_margin)
      addSoft(Constraint(4,_vars), 1e5); // margin
      hard_inequality.add(Constraint(8,_vars)); //limit

      QrTask::init();

      _jacobian.setZero(2*4, _vars);
      _offset.setZero(8);
      // for(int i = 0; i < 4; i++){
      //   _cost.quadratic(2*i, 2*i) = 1;
      //   _cost.quadratic(2*i+1, 2*i+1) = 3;
      // }
      //
      // _cost.quadratic.block(_vars, _vars, _slack, _slack) = 100000*mwoibn::Matrix::Identity(_slack,_slack);
}

void mgnss::higher_level::SupportShapingV6::_update(){

    _original_task._update();

    for(int i = 0; i < 4; i++){

      mwoibn::Matrix3 toN = _accelerations[i].torus().groundNormal()*_accelerations[i].torus().groundNormal().transpose();
      mwoibn::Matrix3 toPN = mwoibn::Matrix3::Identity() - toN;

      _support_jacobian = _accelerations[i].torus().getJacobianWheel()/_robot.rate();
      // std::cout << "getDependant\n" << _accelerations[i].getDependant() << std::endl;
      // std::cout << "_support_jacobian\n" <<  _support_jacobian << std::endl;
      // std::cout << "angular().getWorld()\n" << _accelerations[i].torus().wheelVelocity().angular().getWorld().transpose() << std::endl;
      // std::cout << "getIndependant\n" << _accelerations[i].getIndependant() << std::endl;
      // std::cout << "_support_offset\n" <<  _support_offset.transpose() << std::endl;

      _support_offset =    (_accelerations[i].getDependant()*toPN -_support_jacobian )*_accelerations[i].torus().wheelVelocity().angular().getWorld();
      _support_offset += _accelerations[i].getIndependant();

      _support_jacobian += _accelerations[i].getDependant()*toN;

      _jacobian.middleRows<2>(2*i) = _support_jacobian.topRows<2>();
      _offset.segment<2>(2*i) = _support_offset.head<2>();
    }



    _cost.quadratic.block(0, 0, 8, _vars) = _jacobian.transpose()*_original_task.cost().quadratic.block(0, 0, 8, _vars)*_jacobian;
    _cost.quadratic.block(0, 0, 8, _vars) = _jacobian.transpose()*_original_task.cost().quadratic.block(0, 0, 8, _vars)*_jacobian;
    _cost.quadratic.block(_vars,_vars,_slack, _slack) = _original_task.cost().quadratic.block(_original_task.vars(), _original_task.vars(), _slack, _slack);

    _cost.linear.segment(0,8) = 4*_offset.transpose()*_original_task.cost().quadratic.block(0, 0, 8, _vars)*_jacobian;
    _cost.linear += _original_task.cost().linear;

    std::cout << "SupportShapingV6::_cost.linear\t" << _cost.linear.transpose() << std::endl;
    std::cout << "SupportShapingV6::_cost.quadratic\t" << _cost.quadratic << std::endl;
}

//
void mgnss::higher_level::SupportShapingV6::_outputTransform(){

  // std::cout << "_cost\t" << _optimal_cost << std::endl;
  mwoibn::Vector3 temp__;
  temp__.setZero();
  // std::cout << "_steering_state\t" << _return_state.transpose() << std::endl;

  for(int i = 0; i < 4; i++){
    temp__.head<2>() = _return_state.segment<2>(2*i);
    _return_state.segment<2>(2*i) = (_original_task._wheel_transforms[i]->rotation*temp__).head<2>();
  }

  // std::cout << "_world_state\t" << _return_state.transpose() << std::endl;
}
