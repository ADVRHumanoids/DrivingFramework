#include <mgnss/controllers/steering.h>

void mgnss::events::limit(double b_ref, double& b)
{
//  b -= mwoibn::PI * std::floor((b + mwoibn::HALF_PI) /
//                               mwoibn::PI); // this should normalize to -pi;pi
//  b_ref -= mwoibn::PI * std::floor((b_ref + mwoibn::HALF_PI) / mwoibn::PI);

  mwoibn::eigen_utils::wrapToPi(b);
  mwoibn::eigen_utils::wrapToPi(b_ref);

//  if((std::fabs(b - b_ref) > mwoibn::HALF_PI) &&
//     (std::fabs(b - b_ref) < 3 * mwoibn::HALF_PI))
//    std::cout << b - b_ref << std::endl;

  b = ((std::fabs(b - b_ref) < mwoibn::HALF_PI) ||
       (std::fabs(b - b_ref) > 3 * mwoibn::HALF_PI))
          ? b
          : (b - b_ref < 0) ? b + mwoibn::PI : b - mwoibn::PI;

}

void mgnss::events::limit2(double b_ref, double& b)
{


//  std::cout << b << "\t" << b_ref << "\t";
  b +=   3.14159265 * (std::floor((b_ref + 1.57079632) / 3.14159265) -  std::floor((b + 1.57079632) / 3.14159265));
//  std::cout << b << std::endl;


}


void mgnss::events::limit(const mwoibn::VectorN& b_ref, mwoibn::VectorN& b)
{
  for (int i = 0; i < b.size(); i++)
    limit(b_ref[i], b[i]);
}

void mgnss::events::jointLimits(double& b, double max)
{

  if (b > max) // this is a temporary solution until they are joint
               // limits implemented
    b -= 3.1415926;
  else if (b < -max) // this is a temporary solution until they are
                     // joint limits implemented
    b += 3.1415926;
}

void mgnss::events::jointLimits(mwoibn::VectorN& b, double max)
{

  for (int i = 0; i < b.size(); i++)
    jointLimits(b[i], max);
}

mgnss::events::Steering::Steering(
    mwoibn::robot_class::Robot& robot,
    mwoibn::hierarchical_control::CartesianSimplifiedPelvisTask& plane,
    mwoibn::VectorN init_pose, double K_icm, double K_sp, double dt,
    double margin, double max)
    : _plane(plane), _K_icm(K_icm), _K_sp(K_sp), _dt(dt), _margin(margin),
      _max(max),
      _state(robot.state.state(mwoibn::robot_class::INTERFACE::POSITION))
{
  _v_icm.setZero(_size);
  _b_icm.setZero(_size);
  _v_sp.setZero(_size);
  _b_sp.setZero(_size);
  _b.setZero(_size);
  //     _b_st.setZero(_size);
  _b_st = init_pose;
  _plane_ref.setZero(2);
  _temp.setZero(_size);

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

void mgnss::events::Steering::compute(const mwoibn::Vector3 next_step)
{
  int pow = 4;
  //      std::cout << next_step << std::endl;
  _plane.updateState();
  _heading = _plane.getState()[2];

  _ICM(next_step); // returns a velue in a robot space

  _SPT(); // returns a velue in a robot space

  double l = _margin / _dt;

  std::cout.precision(6);
  std::cout << std::fixed;

  l = std::pow(l, pow);

  //    std::cout << _heading << "\t";
  //    std::cout << next_step[0] << "\t";
  //    std::cout << next_step[1] << "\t";
  //    std::cout << next_step[2] << "\t";

//  bool slow = false;
  for (int i = 0; i < _size; i++)
  {
    double vel = std::fabs(_v_icm[i]*_v_icm[i] + _v_sp[i]*_v_sp[i] + 2*_v_sp[i]*_v_icm[i]*std::cos(_b_icm[i] - _b_icm[i]));
    //      std::cout << _plane.getReference(i)[0] << "\t";
    //      std::cout << _plane.getReference(i)[1] << "\t";

    if (vel < (_margin / _dt))
    {
      //_v_icm[i] = 0;
      //_v_sp[i] = 0;
//      slow = true;
      _b[i] = std::atan2(_K_icm * _v_icm[i] * std::sin(_b_icm[i]) +
                             _K_sp * _v_sp[i] * std::sin(_b_sp[i]),
                         _K_icm * _v_icm[i] * std::cos(_b_icm[i]) +
                             _K_sp * _v_sp[i] * std::cos(_b_sp[i]));

      limit(_b_st[i] - _heading, _b[i]);
      _temp[i] = _b[i] - (_b_st[i] - _heading);

      _b[i] = _b_st[i] - _heading;

      _b[i] += std::pow(vel,pow) / l * _temp[i];

//      std::cout << i << ": " << std::pow(vel,pow) / l << std::endl;

    }
    else
    {
      _b[i] = std::atan2(_K_icm * _v_icm[i] * std::sin(_b_icm[i]) +
                             _K_sp * _v_sp[i] * std::sin(_b_sp[i]),
                         _K_icm * _v_icm[i] * std::cos(_b_icm[i]) +
                             _K_sp * _v_sp[i] * std::cos(_b_sp[i]));
      limit(_b_st[i] - _heading, _b[i]);
    }
    _b_st[i] = _b[i] + _heading; // do give the result in the world frame

  }
//  if(slow)
//    std::cout << "slow down" << std::endl;

}

void mgnss::events::Steering::_ICM(mwoibn::Vector3 next_step)
{

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
    _v_icm[i] = _x * std::cos(_b_icm[i]);
    _v_icm[i] += _y * std::sin(_b_icm[i]);
  }
}

void mgnss::events::Steering::_SPT()
{
  for (int i = 0; i < _size; i++)
    _PT(i);
}

void mgnss::events::Steering::_PT(int i)
{
  // Desired state

  _plane_ref[0] = _plane.getWorldError()[2 * i]; // size 2
  _plane_ref[1] = _plane.getWorldError()[2 * i + 1];

  _b_sp[i] = std::atan2(_plane_ref[1], _plane_ref[0]);

  _v_sp[i] = _plane_ref.norm() / _dt;
}
