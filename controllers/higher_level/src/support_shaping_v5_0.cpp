#include "mgnss/higher_level/support_shaping_v5_0.h"



mgnss::higher_level::SupportShapingV5::SupportShapingV5(mwoibn::robot_class::Robot& robot, YAML::Node config,  std::vector<std::unique_ptr<mwoibn::robot_points::Rotation>>& steering_frames, const mgnss::higher_level::Limit& margin, const mgnss::higher_level::Limit& workspace, const mwoibn::VectorN& spv, const mwoibn::VectorN& beta):
  SupportShapingV4(robot, config, steering_frames, margin, workspace), _spv_desired(spv), _beta_desired(beta){



}

void mgnss::higher_level::SupportShapingV5::_allocate(){

    resize(12,4);
    clear();
    SupportShapingV4::_allocate();
    _cost.quadratic.block(8,8,4,4) = 50*2*mwoibn::Matrix::Identity(4,4);
    // _cost.quadratic.block(_vars, _vars, _slack, _slack) = 100000*mwoibn::Matrix::Identity(_slack,_slack);
    std::cout << "_cost.quadratic\n" << _cost.quadratic << std::endl;
}

void mgnss::higher_level::SupportShapingV5::_update(){

    SupportShapingV4::_update();

    _cost.linear.head<8>() = -2*_spv_desired;
    _cost.linear.segment<4>(8) = -2*_beta_desired;
    _cost.linear = _cost.linear.cwiseProduct(_cost.quadratic.diagonal()/2);

    std::cout << "SupportShapingV5::_cost.linear\t" << _cost.linear.transpose() << std::endl;
}

//
void mgnss::higher_level::SupportShapingV5::_outputTransform(){

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
