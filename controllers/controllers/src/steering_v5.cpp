#include <mgnss/controllers/steering_v5.h>
#include <mgnss/controllers/steering.h>

mgnss::events::Steering5::Steering5(
    mwoibn::robot_class::Robot& robot,
    mwoibn::hierarchical_control::CartesianFlatReferenceTask3& plane,
    mwoibn::VectorN init_pose, double K_icm, double K_sp, double dt,
    double margin, double max)
    : _plane(plane), _K_icm(K_icm), _K_sp(K_sp), _dt(dt), _margin(margin),
      _max(max),
      _state(robot.state.state(mwoibn::robot_class::INTERFACE::POSITION))
{
  _v_icm.setZero(_size);
  _b_icm.setZero(_size);
  _pb_icm.setZero(_size);
  _v_sp.setZero(_size);
  _b_sp.setZero(_size);
  _pb_sp.setZero(_size);
  _b.setZero(_size);
  //     _b_st.setZero(_size);
  _b_st = init_pose;
  _plane_ref.setZero(2);
//  _temp.setZero(_size);
  _damp_sp.setZero(_size);
  _damp_icm.setZero(_size);
  _resteer.setConstant(_size, false);
  _steer.setConstant(_size, false);

  _treshhold = _margin/_dt;

  std::vector<std::string> names = {"hip1_1",   "hip1_2",   "hip1_3",
                                    "hip1_4",   "ankle2_1", "ankle2_2",
                                    "ankle2_3", "ankle2_4"};

  _dofs.resize(_size);

  for (int i = 0; i < _size; i++)
    _dofs[i] = robot.getDof(names[_size + i])[0];

  //    std::cout << "time\t"
  //              << "heading\t"
  //              << "next_x\t"
  //              << "next_y\t"
  //              << "next_th\t"
  //              << "des_x_1\t"
  //              << "des_y_1\t"
  //              << "b_st_1\t"
  //              << "new_1\t"
  //              << "b_icm_1\t"
  //              << "v_icm_1\t"
  //              << "b_sp_1\t"
  //              << "v_sp_1\t"
  //              << "factor_1\t"
  //              << "final_1\t"
  //              << "des_x_2\t"
  //              << "des_y_2\t"
  //              << "b_st_2\t"
  //              << "new_2\t"
  //              << "b_icm_2\t"
  //              << "v_icm_2\t"
  //              << "b_sp_2\t"
  //              << "v_sp_2\t"
  //              << "factor_2\t"
  //              << "final_2\t"
  //              << "des_x_3\t"
  //              << "des_y_3\t"
  //              << "b_st_3\t"
  //              << "new_3\t"
  //              << "b_icm_3\t"
  //              << "v_icm_3\t"
  //              << "b_sp_3\t"
  //              << "v_sp_3\t"
  //              << "factor_3\t"
  //              << "final_3\t"
  //              << "des_x_4\t"
  //              << "des_y_4\t"
  //              << "b_st_4\t"
  //              << "new_4\t"
  //              << "b_icm_4\t"
  //              << "v_icm_4\t"
  //              << "b_sp_4\t"
  //              << "v_sp_4\t"
  //              << "factor_4\t"
  //              << "final_4\t"
  //              << std::endl;
}

//void mgnss::events::Steering5::compute(const mwoibn::Vector3 next_step)
//{
//  int pow = 4;
//  //      std::cout << next_step << std::endl;
//  _plane.updateState();
//  _plane.updateError();

//  _heading = _plane.getState()[2];

//  _ICM(next_step); // returns a velue in a robot space

//  _SPT(); // returns a velue in a robot space

//  double l = _margin / _dt;

//  std::cout.precision(6);
//  std::cout << std::fixed;

//  l = std::pow(l, pow);

//  //    std::cout << _heading << "\t";
//  //    std::cout << next_step[0] << "\t";
//  //    std::cout << next_step[1] << "\t";
//  //    std::cout << next_step[2] << "\t";

//  //  bool slow = false;
//  for (int i = 0; i < _size; i++)
//  {
////    double vel =
////        std::fabs(_v_icm[i] * _v_icm[i] + _v_sp[i] * _v_sp[i] +
////                  2 * _v_sp[i] * _v_icm[i] * std::cos(_b_icm[i] - _b_sp[i]));
//    //      std::cout << _plane.getReference(i)[0] << "\t";
//    //      std::cout << _plane.getReference(i)[1] << "\t";

////    if (vel < (_margin / _dt))
////    {
////      //_v_icm[i] = 0;
////      //_v_sp[i] = 0;
////      //      slow = true;
////      _b[i] = std::atan2(_K_icm * _v_icm[i] * std::sin(_b_icm[i]) +
////                             _K_sp * _v_sp[i] * std::sin(_b_sp[i]),
////                         _K_icm * _v_icm[i] * std::cos(_b_icm[i]) +
////                             _K_sp * _v_sp[i] * std::cos(_b_sp[i]));

////      limit(_b_st[i] - _heading, _b[i]);
////      _temp[i] = _b[i] - (_b_st[i] - _heading);

////      _b[i] = _b_st[i] - _heading;

////      _b[i] += std::pow(vel, pow) / l * _temp[i];

////      //      std::cout << i << ": " << std::pow(vel,pow) / l << std::endl;
////    }
////    else
////    {
//      _b[i] = std::atan2(_K_icm * _v_icm[i] * std::sin(_b_icm[i]) +
//                             _K_sp * _v_sp[i] * std::sin(_b_sp[i]),
//                         _K_icm * _v_icm[i] * std::cos(_b_icm[i]) +
//                             _K_sp * _v_sp[i] * std::cos(_b_sp[i]));
////      limit(_b_st[i] - _heading, _b[i]);
//      mwoibn::eigen_utils::limitHalfPi(_b_st[i] - _heading, _b[i]);
////    }
//    _b_st[i] = _b[i] + _heading; // do give the result in the world frame
//  }

//  //  if(slow)
//  //    std::cout << "slow down" << std::endl;
//}

void mgnss::events::Steering5::compute2(const mwoibn::Vector3 next_step)
{
  _plane.updateState();
  _plane.updateError();

  _heading = _plane.getState()[2];

  _ICM(next_step); // returns a velue in a robot space

  _SPT(); // returns a velue in a robot space


  for (int i = 0; i < _size; i++)
  {
    if(_resteer[i]) _b_st[i] -= mwoibn::PI;

    double vel =
        std::fabs(_v_icm[i] * _v_icm[i] + _v_sp[i] * _v_sp[i] +
                  2 * _v_sp[i] * _v_icm[i] * std::cos(_b_icm[i] - _b_sp[i]));

    _b[i] = std::atan2(_K_icm * _v_icm[i] * std::sin(_b_icm[i]) +
                           _K_sp * _v_sp[i] * std::sin(_b_sp[i]),
                       _K_icm * _v_icm[i] * std::cos(_b_icm[i]) +
                           _K_sp * _v_sp[i] * std::cos(_b_sp[i]));

    _b[i] += _heading;

    _b[i] += 6.28318531 * ( std::floor(_b_st[i] / 6.28318531) -  std::floor(_b[i] / 6.28318531));

    limit2PI(_b_st[i], _b[i]);

    _b_st[i] += std::tanh(std::fabs(vel) / _treshhold) * (_b[i] - _b_st[i]); // do give the result in the world frame
  }
}

void mgnss::events::Steering5::limit2PI(double ref, double& st){

  if(st - ref > mwoibn::HALF_PI){
//    std::cout << "\t" << ref << "\t" << st << "\t";

    st -= mwoibn::PI;
//  std::cout << st << std::endl;
   limit2PI(ref, st);
  }
  else if ((ref - st > mwoibn::HALF_PI)){
//    std::cout << "\t" << ref << "\t" << st << "\t";
    st += mwoibn::PI;
//    std::cout << st << std::endl;
    limit2PI(ref, st);
  }
}

void mgnss::events::Steering5::limitPI(double ref, double& st){

  if(st - ref > mwoibn::HALF_PI/2){
//    std::cout << "\t" << ref << "\t" << st << "\t";

    st -= mwoibn::HALF_PI;
//  std::cout << st << std::endl;
   limitPI(ref, st);
  }
  else if ((ref - st > mwoibn::HALF_PI/2)){
//    std::cout << "\t" << ref << "\t" << st << "\t";
    st += mwoibn::HALF_PI;
//    std::cout << st << std::endl;
    limitPI(ref, st);
  }
}

void mgnss::events::Steering5::_ICM(mwoibn::Vector3 next_step)
{

  _pb_icm.noalias() = _b_icm;

  for (int i = 0; i < _size; i++)
  {
    _plane_ref.noalias() = _plane.getPointStateReference(i).head(2);

    _x = std::cos(_heading) * next_step[0];
    _x += std::sin(_heading) * next_step[1];
    _x -= _plane_ref[1] * next_step[2];
    _y = -std::sin(_heading) * next_step[0];
    _y += std::cos(_heading) * next_step[1];
    _y += _plane_ref[0] * next_step[2];

    _b_icm[i] = std::atan2(_y, _x); // this uses atan2 to avoid
                                    // singularities in a atan
                                    // implementation

//    double b_last = _b_icm[i];
//    mwoibn::eigen_utils::limitHalfPi(_pb_icm[i], _b_icm[i]);
//    limitPI(_pb_icm[i],_b_icm[i]);


    mwoibn::eigen_utils::limitHalfPi(_b_icm[i], _pb_icm[i]);
    limit2PI(_b_icm[i], _pb_icm[i]);

    double v_last = _v_icm[i];

    _v_icm[i] = _x * std::cos(_b_icm[i]);
    _v_icm[i] += _y * std::sin(_b_icm[i]);

//    b_last -= _b_icm[i];
//    mwoibn::eigen_utils::wrapToPi(b_last);
//    if( b_last > 0.0001)
//      _v_icm[i] = -_v_icm[i];

    _damp_icm[i] = std::tanh(std::fabs(_v_icm[i]) / _treshhold);


    _b_icm[i] = _pb_icm[i] + _damp_icm[i] * (_b_icm[i] - _pb_icm[i]);
    _v_icm[i] = v_last + _damp_icm[i] * (_v_icm[i] - v_last);

  }
}

void mgnss::events::Steering5::_SPT()
{
  for (int i = 0; i < _size; i++)
    _PT(i);
}

void mgnss::events::Steering5::_PT(int i)
{
  // Desired state

  _pb_sp[i] = _b_sp[i];

  _plane_ref.noalias() = _plane.getReferenceError(i).head(2); // size 2

  _b_sp[i] = std::atan2(_plane_ref[1], _plane_ref[0]);

//  double b_last = _b_sp[i];
  double v_last = _v_sp[i];

//  mwoibn::eigen_utils::limitHalfPi(_pb_sp[i], _b_sp[i]);
//  limitPI(_pb_sp[i], _b_sp[i]);
  mwoibn::eigen_utils::limitHalfPi(_b_sp[i], _pb_sp[i]);
  limit2PI(_b_sp[i], _pb_sp[i]);


  _v_sp[i] = _plane_ref.norm() / _dt;

//  b_last -= _b_sp[i];
//  mwoibn::eigen_utils::wrapToPi(b_last);
//  if(b_last > 0.0001)
//    _v_icm[i] = -_v_icm[i];

  _damp_sp[i] = std::tanh(std::fabs(_v_sp[i]) / _treshhold);
  _b_sp[i] = _pb_sp[i] + _damp_sp[i] *(_b_sp[i] - _pb_sp[i]);
  _v_sp[i] = v_last + _damp_sp[i] * (_v_sp[i] - v_last);
}
