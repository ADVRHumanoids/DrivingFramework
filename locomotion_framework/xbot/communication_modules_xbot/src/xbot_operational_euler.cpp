#include "mwoibn/communication_modules/xbot_operational_euler.h"

mwoibn::communication_modules::XBotOperationalEuler::XBotOperationalEuler(
    mwoibn::robot_class::State& command, mwoibn::robot_class::BiMap map,
    YAML::Node config, XBot::RobotInterface& robot, double rate)
    : BasicOperationalEuler(command, map, config), _rate(rate)
{

  std::map<std::string, XBot::ImuSensor::ConstPtr> imus = robot.getImu();

  if(!map.reversed().size()) _is_static = false;
  else _is_static = true;

  if (!config["sensor"] && imus.size() == 1)
  {
    _imu = imus.begin()->second;
    std::cout
        << "Unknown imu name while only one imu found, load imu called: " +
               imus.begin()->first << std::endl;
  }
  else if (!config["sensor"] && imus.size() > 1)
    throw(std::invalid_argument(
        "More than one imu found. Please define name of the imu sensor to be "
        "loaded.\n")); // if there is only one
                       // sensor this could be
                       // automatic
  else if (imus.size() == 0)
    throw(std::invalid_argument(
        "Couldn't find any imu sensor.")); // if there is only one
                                           // sensor this could be
                                           // automatic
  else
    _imu = imus[config["sensor"].as<std::string>()];

  //  _imu = _robot->getImu(config["name"].as<std::string>());

  if (!_imu)
    throw(std::invalid_argument(
        "Couldn't find an IMU sensor named " +
        config["sensor"].as<std::string>())); // if there is only one sensor this
                                            // could be automatic

  _base.setZero(_size);

  std::cout << "offset position\n" << _offset_position << std::endl;
  std::cout << "offset orientation\n" << _offset_orientation << std::endl;

  _linear_state << 0,0,0; // position estimation is not supported

//  _rotation << 1,0,0,0,1,0,0,0,1;
  _offset_org = _offset_orientation;

//  BasicOperationalEuler::getPosition(_rotation, _linear_state);
  std::cout << "Loaded xbot operational feedback " << config["name"] << std::endl;

//  std::cout <<_map_dofs << std::endl;

}

bool mwoibn::communication_modules::XBotOperationalEuler::get()
{

  if(!_size) return true;

  _imu->getOrientation(_rotation);

  getPosition(_rotation, _linear_state);
  return true;

}

void mwoibn::communication_modules::XBotOperationalEuler::getPosition(mwoibn::Matrix3 orientation,
                           mwoibn::Vector3 position)
  {

    if(!_initialized) reset();

    _base.tail(3) =
        (_offset_orientation*orientation)
            .eulerAngles(_angels[0], _angels[1],
                         _angels[2]); // Check if the convention is met here

    //_base.tail<1>()[0] -= _offset_z;
    _command.set(_base, _map_dofs, mwoibn::robot_class::INTERFACE::POSITION);
  }

bool mwoibn::communication_modules::XBotOperationalEuler::reset(){

  if(!_size) {
    _initialized = true;
    return _initialized;
  }
/*
 std::cout << "rotation" << std::endl;
 std::cout << _rotation << std::endl;
*/
/*
 std::cout << "x times y " << std::endl;
 std::cout << _rotation.col(0).cross(_rotation.col(1)) << std::endl;
 std::cout << "y times z " << std::endl;
 std::cout << _rotation.col(1).cross(_rotation.col(2)) << std::endl;
 std::cout << "z times x " << std::endl;
 std::cout << _rotation.col(2).cross(_rotation.col(0)) << std::endl;
*/
/*
 std::cout << "_offset_org" << std::endl;
 std::cout << _offset_org << std::endl;
*/
 _rot_z = _offset_org*_rotation;
/*
 std::cout << "_rot_z" << std::endl;
 std::cout << _rot_z << std::endl;
*/
/*
 mwoibn::Matrix3 test_1, test_2, test_3, test_4, test_5, test_6, test_7, test_8;

  double p_xx = _rot_z(0,0);
  double p_xy = _rot_z(0,1);
  double p_xz = _rot_z(0,2);
  double p_yx = _rot_z(1,0);
  double p_yy = _rot_z(1,1);
  double p_yz = _rot_z(1,2);
  double p_zx = _rot_z(2,0);
  double p_zy = _rot_z(2,1);
  double p_zz = _rot_z(2,2);

  double r_xx, r_xy, r_xz, r_yx, r_yy, r_yz, r_zx, r_zy, r_zz;

  // 1 solution
  r_zz =  (p_xx*p_yy*p_zz - p_xy*p_yx*p_zz - p_xy*p_yz*std::sqrt(-p_zz*p_zz + 1) + p_xz*p_yy*std::sqrt(-p_zz*p_zz + 1))/(p_xx*p_yy*p_zz - p_xx*p_yz*p_zy - p_xy*p_yx*p_zz + p_xy*p_yz*p_zx + p_xz*p_yx*p_zy - p_xz*p_yy*p_zx);
  r_yz = (-p_xx*p_yz + p_xz*p_yx)/(p_xx*p_yy*p_zz - p_xx*p_yz*p_zy - p_xy*p_yx*p_zz + p_xy*p_yz*p_zx + p_xz*p_yx*p_zy - p_xz*p_yy*p_zx);
  r_zy = (-p_xy*p_zz*r_zz + p_xy*p_zz + p_xz*p_zy*r_zz)/(p_xy*p_yz - p_xz*p_yy);
  r_zx = (-p_yz*r_zy - p_zz*r_zz + p_zz)/p_xz;
  r_yy = -r_yz*(p_xx*p_zz - p_xz*p_zx)/(p_xx*p_yz - p_xz*p_yx);
  r_yx = -(p_yx*r_yy + p_zx*r_yz)/p_xx;
  r_xz = (-p_xx*p_xx*p_yy*p_yy*p_zz*r_zz + p_xx*p_xx*p_yy*p_yy*p_zz + p_xx*p_xx*p_yy*p_yz*p_zy*r_zz + 2*p_xx*p_xy*p_yx*p_yy*p_zz*r_zz - 2*p_xx*p_xy*p_yx*p_yy*p_zz - p_xx*p_xy*p_yx*p_yz*p_zy*r_zz - p_xx*p_xy*p_yy*p_yz*p_zx*r_zz - p_xx*p_xz*p_yx*p_yy*p_zy*r_zz + p_xx*p_xz*p_yy*p_yy*p_zx*r_zz - p_xy*p_xy*p_yx*p_yx*p_zz*r_zz + p_xy*p_xy*p_yx*p_yx*p_zz + p_xy*p_xy*p_yx*p_yz*p_zx*r_zz + p_xy*p_xy*p_yz*p_yz*p_zz + p_xy*p_xz*p_yx*p_yx*p_zy*r_zz - p_xy*p_xz*p_yx*p_yy*p_zx*r_zz - 2*p_xy*p_xz*p_yy*p_yz*p_zz + p_xz*p_xz*p_yy*p_yy*p_zz)/(p_xx*p_xy*p_yy*p_yz*p_zz - p_xx*p_xy*p_yz*p_yz*p_zy - p_xx*p_xz*p_yy*p_yy*p_zz + p_xx*p_xz*p_yy*p_yz*p_zy - p_xy*p_xy*p_yx*p_yz*p_zz + p_xy*p_xy*p_yz*p_yz*p_zx + p_xy*p_xz*p_yx*p_yy*p_zz + p_xy*p_xz*p_yx*p_yz*p_zy - 2*p_xy*p_xz*p_yy*p_yz*p_zx - p_xz*p_xz*p_yx*p_yy*p_zy + p_xz*p_xz*p_yy*p_yy*p_zx);
  r_xy = (-p_xx*p_xx*p_yy*p_zy*r_xz*r_yy - p_xx*p_xx*p_zy*p_zy*r_xz*r_yz + p_xx*p_xy*p_yx*p_zy*r_xz*r_yy + p_xx*p_xy*p_yy*p_zx*r_xz*r_yy + 2*p_xx*p_xy*p_zx*p_zy*r_xz*r_yz - p_xx*p_xy*p_zz - p_xy*p_xy*p_yx*p_zx*r_xz*r_yy - p_xy*p_xy*p_zx*p_zx*r_xz*r_yz)/(p_xx*p_xx*p_yy*p_yy*r_yy + p_xx*p_xx*p_yy*p_zy*r_yz - 2*p_xx*p_xy*p_yx*p_yy*r_yy - p_xx*p_xy*p_yx*p_zy*r_yz - p_xx*p_xy*p_yy*p_zx*r_yz + p_xy*p_xy*p_yx*p_yx*r_yy + p_xy*p_xy*p_yx*p_zx*r_yz);
  r_xx = -(p_yy*r_xy + p_zy*r_xz)/p_xy;

  test_1 << r_xx, r_xy, r_xz, r_yx, r_yy, r_yz, r_zx, r_zy, r_zz;

  // 2 solution
  r_zz = (p_xx*p_yy*p_zz - p_xy*p_yx*p_zz + p_xy*p_yz*std::sqrt(-p_zz*p_zz + 1) - p_xz*p_yy*std::sqrt(-p_zz*p_zz + 1))/(p_xx*p_yy*p_zz - p_xx*p_yz*p_zy - p_xy*p_yx*p_zz + p_xy*p_yz*p_zx + p_xz*p_yx*p_zy - p_xz*p_yy*p_zx);
  r_yz = (-p_xx*p_yz + p_xz*p_yx)/(p_xx*p_yy*p_zz - p_xx*p_yz*p_zy - p_xy*p_yx*p_zz + p_xy*p_yz*p_zx + p_xz*p_yx*p_zy - p_xz*p_yy*p_zx);
  r_zy = (-p_xy*p_zz*r_zz + p_xy*p_zz + p_xz*p_zy*r_zz)/(p_xy*p_yz - p_xz*p_yy);
  r_zx = (-p_yz*r_zy - p_zz*r_zz + p_zz)/p_xz;
  r_yy = -r_yz*(p_xx*p_zz - p_xz*p_zx)/(p_xx*p_yz - p_xz*p_yx);
  r_yx = -(p_yx*r_yy + p_zx*r_yz)/p_xx;
  r_xz = (-p_xx*p_xx*p_yy*p_yy*p_zz*r_zz + p_xx*p_xx*p_yy*p_yy*p_zz + p_xx*p_xx*p_yy*p_yz*p_zy*r_zz + 2*p_xx*p_xy*p_yx*p_yy*p_zz*r_zz - 2*p_xx*p_xy*p_yx*p_yy*p_zz - p_xx*p_xy*p_yx*p_yz*p_zy*r_zz - p_xx*p_xy*p_yy*p_yz*p_zx*r_zz - p_xx*p_xz*p_yx*p_yy*p_zy*r_zz + p_xx*p_xz*p_yy*p_yy*p_zx*r_zz - p_xy*p_xy*p_yx*p_yx*p_zz*r_zz + p_xy*p_xy*p_yx*p_yx*p_zz + p_xy*p_xy*p_yx*p_yz*p_zx*r_zz + p_xy*p_xy*p_yz*p_yz*p_zz + p_xy*p_xz*p_yx*p_yx*p_zy*r_zz - p_xy*p_xz*p_yx*p_yy*p_zx*r_zz - 2*p_xy*p_xz*p_yy*p_yz*p_zz + p_xz*p_xz*p_yy*p_yy*p_zz)/(p_xx*p_xy*p_yy*p_yz*p_zz - p_xx*p_xy*p_yz*p_yz*p_zy - p_xx*p_xz*p_yy*p_yy*p_zz + p_xx*p_xz*p_yy*p_yz*p_zy - p_xy*p_xy*p_yx*p_yz*p_zz + p_xy*p_xy*p_yz*p_yz*p_zx + p_xy*p_xz*p_yx*p_yy*p_zz + p_xy*p_xz*p_yx*p_yz*p_zy - 2*p_xy*p_xz*p_yy*p_yz*p_zx - p_xz*p_xz*p_yx*p_yy*p_zy + p_xz*p_xz*p_yy*p_yy*p_zx);
  r_xy = (-p_xx*p_xx*p_yy*p_zy*r_xz*r_yy - p_xx*p_xx*p_zy*p_zy*r_xz*r_yz + p_xx*p_xy*p_yx*p_zy*r_xz*r_yy + p_xx*p_xy*p_yy*p_zx*r_xz*r_yy + 2*p_xx*p_xy*p_zx*p_zy*r_xz*r_yz - p_xx*p_xy*p_zz - p_xy*p_xy*p_yx*p_zx*r_xz*r_yy - p_xy*p_xy*p_zx*p_zx*r_xz*r_yz)/(p_xx*p_xx*p_yy*p_yy*r_yy + p_xx*p_xx*p_yy*p_zy*r_yz - 2*p_xx*p_xy*p_yx*p_yy*r_yy - p_xx*p_xy*p_yx*p_zy*r_yz - p_xx*p_xy*p_yy*p_zx*r_yz + p_xy*p_xy*p_yx*p_yx*r_yy + p_xy*p_xy*p_yx*p_zx*r_yz);
  r_xx = -(p_yy*r_xy + p_zy*r_xz)/p_xy;

  test_2 << r_xx, r_xy, r_xz, r_yx, r_yy, r_yz, r_zx, r_zy, r_zz;

  // 3 solution

  r_zz = (p_xx*p_yy*p_zz - p_xy*p_yx*p_zz - p_xy*p_yz*std::sqrt(-p_zz*p_zz + 1) + p_xz*p_yy*std::sqrt(-p_zz*p_zz + 1))/(p_xx*p_yy*p_zz - p_xx*p_yz*p_zy - p_xy*p_yx*p_zz + p_xy*p_yz*p_zx + p_xz*p_yx*p_zy - p_xz*p_yy*p_zx);
  r_yz = (p_xx*p_yz - p_xz*p_yx)/(p_xx*p_yy*p_zz - p_xx*p_yz*p_zy - p_xy*p_yx*p_zz + p_xy*p_yz*p_zx + p_xz*p_yx*p_zy - p_xz*p_yy*p_zx);
  r_zy = (-p_xy*p_zz*r_zz + p_xy*p_zz + p_xz*p_zy*r_zz)/(p_xy*p_yz - p_xz*p_yy);
  r_zx = (-p_yz*r_zy - p_zz*r_zz + p_zz)/p_xz;
  r_yy = -r_yz*(p_xx*p_zz - p_xz*p_zx)/(p_xx*p_yz - p_xz*p_yx);
  r_yx = -(p_yx*r_yy + p_zx*r_yz)/p_xx;
  r_xz = (p_xx*p_xx*p_yy*p_yy*p_zz*r_zz - p_xx*p_xx*p_yy*p_yy*p_zz - p_xx*p_xx*p_yy*p_yz*p_zy*r_zz - 2*p_xx*p_xy*p_yx*p_yy*p_zz*r_zz + 2*p_xx*p_xy*p_yx*p_yy*p_zz + p_xx*p_xy*p_yx*p_yz*p_zy*r_zz + p_xx*p_xy*p_yy*p_yz*p_zx*r_zz + p_xx*p_xz*p_yx*p_yy*p_zy*r_zz - p_xx*p_xz*p_yy*p_yy*p_zx*r_zz + p_xy*p_xy*p_yx*p_yx*p_zz*r_zz - p_xy*p_xy*p_yx*p_yx*p_zz - p_xy*p_xy*p_yx*p_yz*p_zx*r_zz - p_xy*p_xy*p_yz*p_yz*p_zz - p_xy*p_xz*p_yx*p_yx*p_zy*r_zz + p_xy*p_xz*p_yx*p_yy*p_zx*r_zz + 2*p_xy*p_xz*p_yy*p_yz*p_zz - p_xz*p_xz*p_yy*p_yy*p_zz)/(p_xx*p_xy*p_yy*p_yz*p_zz - p_xx*p_xy*p_yz*p_yz*p_zy - p_xx*p_xz*p_yy*p_yy*p_zz + p_xx*p_xz*p_yy*p_yz*p_zy - p_xy*p_xy*p_yx*p_yz*p_zz + p_xy*p_xy*p_yz*p_yz*p_zx + p_xy*p_xz*p_yx*p_yy*p_zz + p_xy*p_xz*p_yx*p_yz*p_zy - 2*p_xy*p_xz*p_yy*p_yz*p_zx - p_xz*p_xz*p_yx*p_yy*p_zy + p_xz*p_xz*p_yy*p_yy*p_zx);
  r_xy = (-p_xx*p_xx*p_yy*p_zy*r_xz*r_yy - p_xx*p_xx*p_zy*p_zy*r_xz*r_yz + p_xx*p_xy*p_yx*p_zy*r_xz*r_yy + p_xx*p_xy*p_yy*p_zx*r_xz*r_yy + 2*p_xx*p_xy*p_zx*p_zy*r_xz*r_yz - p_xx*p_xy*p_zz - p_xy*p_xy*p_yx*p_zx*r_xz*r_yy - p_xy*p_xy*p_zx*p_zx*r_xz*r_yz)/(p_xx*p_xx*p_yy*p_yy*r_yy + p_xx*p_xx*p_yy*p_zy*r_yz - 2*p_xx*p_xy*p_yx*p_yy*r_yy - p_xx*p_xy*p_yx*p_zy*r_yz - p_xx*p_xy*p_yy*p_zx*r_yz + p_xy*p_xy*p_yx*p_yx*r_yy + p_xy*p_xy*p_yx*p_zx*r_yz);
  r_xx = -(p_yy*r_xy + p_zy*r_xz)/p_xy;

  test_3 << r_xx, r_xy, r_xz, r_yx, r_yy, r_yz, r_zx, r_zy, r_zz;

  // 4 solution

  r_zz = (p_xx*p_yy*p_zz - p_xy*p_yx*p_zz + p_xy*p_yz*std::sqrt(-p_zz*p_zz + 1) - p_xz*p_yy*std::sqrt(-p_zz*p_zz + 1))/(p_xx*p_yy*p_zz - p_xx*p_yz*p_zy - p_xy*p_yx*p_zz + p_xy*p_yz*p_zx + p_xz*p_yx*p_zy - p_xz*p_yy*p_zx);
  r_yz = (p_xx*p_yz - p_xz*p_yx)/(p_xx*p_yy*p_zz - p_xx*p_yz*p_zy - p_xy*p_yx*p_zz + p_xy*p_yz*p_zx + p_xz*p_yx*p_zy - p_xz*p_yy*p_zx);
  r_zy = (-p_xy*p_zz*r_zz + p_xy*p_zz + p_xz*p_zy*r_zz)/(p_xy*p_yz - p_xz*p_yy);
  r_zx = (-p_yz*r_zy - p_zz*r_zz + p_zz)/p_xz;
  r_yy = -r_yz*(p_xx*p_zz - p_xz*p_zx)/(p_xx*p_yz - p_xz*p_yx);
  r_yx = -(p_yx*r_yy + p_zx*r_yz)/p_xx;
  r_xz = (p_xx*p_xx*p_yy*p_yy*p_zz*r_zz - p_xx*p_xx*p_yy*p_yy*p_zz - p_xx*p_xx*p_yy*p_yz*p_zy*r_zz - 2*p_xx*p_xy*p_yx*p_yy*p_zz*r_zz + 2*p_xx*p_xy*p_yx*p_yy*p_zz + p_xx*p_xy*p_yx*p_yz*p_zy*r_zz + p_xx*p_xy*p_yy*p_yz*p_zx*r_zz + p_xx*p_xz*p_yx*p_yy*p_zy*r_zz - p_xx*p_xz*p_yy*p_yy*p_zx*r_zz + p_xy*p_xy*p_yx*p_yx*p_zz*r_zz - p_xy*p_xy*p_yx*p_yx*p_zz - p_xy*p_xy*p_yx*p_yz*p_zx*r_zz - p_xy*p_xy*p_yz*p_yz*p_zz - p_xy*p_xz*p_yx*p_yx*p_zy*r_zz + p_xy*p_xz*p_yx*p_yy*p_zx*r_zz + 2*p_xy*p_xz*p_yy*p_yz*p_zz - p_xz*p_xz*p_yy*p_yy*p_zz)/(p_xx*p_xy*p_yy*p_yz*p_zz - p_xx*p_xy*p_yz*p_yz*p_zy - p_xx*p_xz*p_yy*p_yy*p_zz + p_xx*p_xz*p_yy*p_yz*p_zy - p_xy*p_xy*p_yx*p_yz*p_zz + p_xy*p_xy*p_yz*p_yz*p_zx + p_xy*p_xz*p_yx*p_yy*p_zz + p_xy*p_xz*p_yx*p_yz*p_zy - 2*p_xy*p_xz*p_yy*p_yz*p_zx - p_xz*p_xz*p_yx*p_yy*p_zy + p_xz*p_xz*p_yy*p_yy*p_zx);
  r_xy = (-p_xx*p_xx*p_yy*p_zy*r_xz*r_yy - p_xx*p_xx*p_zy*p_zy*r_xz*r_yz + p_xx*p_xy*p_yx*p_zy*r_xz*r_yy + p_xx*p_xy*p_yy*p_zx*r_xz*r_yy + 2*p_xx*p_xy*p_zx*p_zy*r_xz*r_yz - p_xx*p_xy*p_zz - p_xy*p_xy*p_yx*p_zx*r_xz*r_yy - p_xy*p_xy*p_zx*p_zx*r_xz*r_yz)/(p_xx*p_xx*p_yy*p_yy*r_yy + p_xx*p_xx*p_yy*p_zy*r_yz - 2*p_xx*p_xy*p_yx*p_yy*r_yy - p_xx*p_xy*p_yx*p_zy*r_yz - p_xx*p_xy*p_yy*p_zx*r_yz + p_xy*p_xy*p_yx*p_yx*r_yy + p_xy*p_xy*p_yx*p_zx*r_yz);
  r_xx = -(p_yy*r_xy + p_zy*r_xz)/p_xy;

  test_4 << r_xx, r_xy, r_xz, r_yx, r_yy, r_yz, r_zx, r_zy, r_zz;

  // 5 solution

  r_yx = -(p_yx*r_yy + p_zx*r_yz)/p_xx;
  r_yy = (-p_xx*p_zy*r_yz - p_xx*p_zz + p_xy*p_zx*r_yz)/(p_xx*p_yy - p_xy*p_yx);
  r_yz = (p_xx*p_yy*std::sqrt(-p_zz*p_zz + 1) + p_xx*p_yz*p_zz - p_xy*p_yx*std::sqrt(-p_zz*p_zz + 1) - p_xz*p_yx*p_zz)/(p_xx*p_yy*p_zz - p_xx*p_yz*p_zy - p_xy*p_yx*p_zz + p_xy*p_yz*p_zx + p_xz*p_yx*p_zy - p_xz*p_yy*p_zx);
  r_zx = (-p_yz*r_zy - p_zz*r_zz + p_zz)/p_xz;
  r_zy = (-p_xx*p_zz*r_zz + p_xx*p_zz + p_xz*p_zx*r_zz)/(p_xx*p_yz - p_xz*p_yx);
  r_zz = (p_xx*p_yy*p_zz - p_xx*p_yz*std::sqrt(-p_zz*p_zz + 1) - p_xy*p_yx*p_zz + p_xz*p_yx*std::sqrt(-p_zz*p_zz + 1))/(p_xx*p_yy*p_zz - p_xx*p_yz*p_zy - p_xy*p_yx*p_zz + p_xy*p_yz*p_zx + p_xz*p_yx*p_zy - p_xz*p_yy*p_zx);
  r_xz = (-p_xy*p_yz + p_xz*p_yy)/(p_xx*p_yy*p_zz - p_xx*p_yz*p_zy - p_xy*p_yx*p_zz + p_xy*p_yz*p_zx + p_xz*p_yx*p_zy - p_xz*p_yy*p_zx);
  r_xy = (-p_xx*p_xx*p_yy*p_zy*r_xz*r_yy - p_xx*p_xx*p_zy*p_zy*r_xz*r_yz + p_xx*p_xy*p_yx*p_zy*r_xz*r_yy + p_xx*p_xy*p_yy*p_zx*r_xz*r_yy + 2*p_xx*p_xy*p_zx*p_zy*r_xz*r_yz - p_xx*p_xy*p_zz - p_xy*p_xy*p_yx*p_zx*r_xz*r_yy - p_xy*p_xy*p_zx*p_zx*r_xz*r_yz)/(p_xx*p_xx*p_yy*p_yy*r_yy + p_xx*p_xx*p_yy*p_zy*r_yz - 2*p_xx*p_xy*p_yx*p_yy*r_yy - p_xx*p_xy*p_yx*p_zy*r_yz - p_xx*p_xy*p_yy*p_zx*r_yz + p_xy*p_xy*p_yx*p_yx*r_yy + p_xy*p_xy*p_yx*p_zx*r_yz);
  r_xx = -(p_yy*r_xy + p_zy*r_xz)/p_xy;

  test_5 << r_xx, r_xy, r_xz, r_yx, r_yy, r_yz, r_zx, r_zy, r_zz;

  // 6 solution

  r_yx = -(p_yx*r_yy + p_zx*r_yz)/p_xx;
  r_yy = (-p_xx*p_zy*r_yz - p_xx*p_zz + p_xy*p_zx*r_yz)/(p_xx*p_yy - p_xy*p_yx);
  r_yz = (-p_xx*p_yy*std::sqrt(-p_zz*p_zz + 1) + p_xx*p_yz*p_zz + p_xy*p_yx*std::sqrt(-p_zz*p_zz + 1) - p_xz*p_yx*p_zz)/(p_xx*p_yy*p_zz - p_xx*p_yz*p_zy - p_xy*p_yx*p_zz + p_xy*p_yz*p_zx + p_xz*p_yx*p_zy - p_xz*p_yy*p_zx);
  r_zx = (-p_yz*r_zy - p_zz*r_zz + p_zz)/p_xz;
  r_zy = (-p_xx*p_zz*r_zz + p_xx*p_zz + p_xz*p_zx*r_zz)/(p_xx*p_yz - p_xz*p_yx);
  r_zz = (p_xx*p_yy*p_zz + p_xx*p_yz*std::sqrt(-p_zz*p_zz + 1) - p_xy*p_yx*p_zz - p_xz*p_yx*std::sqrt(-p_zz*p_zz + 1))/(p_xx*p_yy*p_zz - p_xx*p_yz*p_zy - p_xy*p_yx*p_zz + p_xy*p_yz*p_zx + p_xz*p_yx*p_zy - p_xz*p_yy*p_zx);
  r_xz = (-p_xy*p_yz + p_xz*p_yy)/(p_xx*p_yy*p_zz - p_xx*p_yz*p_zy - p_xy*p_yx*p_zz + p_xy*p_yz*p_zx + p_xz*p_yx*p_zy - p_xz*p_yy*p_zx);
  r_xy = (-p_xx*p_xx*p_yy*p_zy*r_xz*r_yy - p_xx*p_xx*p_zy*p_zy*r_xz*r_yz + p_xx*p_xy*p_yx*p_zy*r_xz*r_yy + p_xx*p_xy*p_yy*p_zx*r_xz*r_yy + 2*p_xx*p_xy*p_zx*p_zy*r_xz*r_yz - p_xx*p_xy*p_zz - p_xy*p_xy*p_yx*p_zx*r_xz*r_yy - p_xy*p_xy*p_zx*p_zx*r_xz*r_yz)/(p_xx*p_xx*p_yy*p_yy*r_yy + p_xx*p_xx*p_yy*p_zy*r_yz - 2*p_xx*p_xy*p_yx*p_yy*r_yy - p_xx*p_xy*p_yx*p_zy*r_yz - p_xx*p_xy*p_yy*p_zx*r_yz + p_xy*p_xy*p_yx*p_yx*r_yy + p_xy*p_xy*p_yx*p_zx*r_yz);
  r_xx = -(p_yy*r_xy + p_zy*r_xz)/p_xy;

  test_6 << r_xx, r_xy, r_xz, r_yx, r_yy, r_yz, r_zx, r_zy, r_zz;

  // 7 solution

  r_yx = -(p_yx*r_yy + p_zx*r_yz)/p_xx;
  r_yy = (-p_xx*p_zy*r_yz + p_xx*p_zz + p_xy*p_zx*r_yz)/(p_xx*p_yy - p_xy*p_yx);
  r_yz = (-p_xx*p_yy*std::sqrt(-p_zz*p_zz + 1) - p_xx*p_yz*p_zz + p_xy*p_yx*std::sqrt(-p_zz*p_zz + 1) + p_xz*p_yx*p_zz)/(p_xx*p_yy*p_zz - p_xx*p_yz*p_zy - p_xy*p_yx*p_zz + p_xy*p_yz*p_zx + p_xz*p_yx*p_zy - p_xz*p_yy*p_zx);
  r_zx = (-p_yz*r_zy - p_zz*r_zz + p_zz)/p_xz;
  r_zy = (-p_xx*p_zz*r_zz + p_xx*p_zz + p_xz*p_zx*r_zz)/(p_xx*p_yz - p_xz*p_yx);
  r_zz = (p_xx*p_yy*p_zz - p_xx*p_yz*std::sqrt(-p_zz*p_zz + 1) - p_xy*p_yx*p_zz + p_xz*p_yx*std::sqrt(-p_zz*p_zz + 1))/(p_xx*p_yy*p_zz - p_xx*p_yz*p_zy - p_xy*p_yx*p_zz + p_xy*p_yz*p_zx + p_xz*p_yx*p_zy - p_xz*p_yy*p_zx);
  r_xz = (p_xy*p_yz - p_xz*p_yy)/(p_xx*p_yy*p_zz - p_xx*p_yz*p_zy - p_xy*p_yx*p_zz + p_xy*p_yz*p_zx + p_xz*p_yx*p_zy - p_xz*p_yy*p_zx);
  r_xy = (-p_xx*p_xx*p_yy*p_zy*r_xz*r_yy - p_xx*p_xx*p_zy*p_zy*r_xz*r_yz + p_xx*p_xy*p_yx*p_zy*r_xz*r_yy + p_xx*p_xy*p_yy*p_zx*r_xz*r_yy + 2*p_xx*p_xy*p_zx*p_zy*r_xz*r_yz - p_xx*p_xy*p_zz - p_xy*p_xy*p_yx*p_zx*r_xz*r_yy - p_xy*p_xy*p_zx*p_zx*r_xz*r_yz)/(p_xx*p_xx*p_yy*p_yy*r_yy + p_xx*p_xx*p_yy*p_zy*r_yz - 2*p_xx*p_xy*p_yx*p_yy*r_yy - p_xx*p_xy*p_yx*p_zy*r_yz - p_xx*p_xy*p_yy*p_zx*r_yz + p_xy*p_xy*p_yx*p_yx*r_yy + p_xy*p_xy*p_yx*p_zx*r_yz);
  r_xx = -(p_yy*r_xy + p_zy*r_xz)/p_xy;

  test_7 << r_xx, r_xy, r_xz, r_yx, r_yy, r_yz, r_zx, r_zy, r_zz;

  // 8 solution

  r_yx = -(p_yx*r_yy + p_zx*r_yz)/p_xx;
  r_yy = (-p_xx*p_zy*r_yz + p_xx*p_zz + p_xy*p_zx*r_yz)/(p_xx*p_yy - p_xy*p_yx);
  r_yz = (p_xx*p_yy*std::sqrt(-p_zz*p_zz + 1) - p_xx*p_yz*p_zz - p_xy*p_yx*std::sqrt(-p_zz*p_zz + 1) + p_xz*p_yx*p_zz)/(p_xx*p_yy*p_zz - p_xx*p_yz*p_zy - p_xy*p_yx*p_zz + p_xy*p_yz*p_zx + p_xz*p_yx*p_zy - p_xz*p_yy*p_zx);
  r_zx = (-p_yz*r_zy - p_zz*r_zz + p_zz)/p_xz;
  r_zy = (-p_xx*p_zz*r_zz + p_xx*p_zz + p_xz*p_zx*r_zz)/(p_xx*p_yz - p_xz*p_yx);
  r_zz = (p_xx*p_yy*p_zz + p_xx*p_yz*std::sqrt(-p_zz*p_zz + 1) - p_xy*p_yx*p_zz - p_xz*p_yx*std::sqrt(-p_zz*p_zz + 1))/(p_xx*p_yy*p_zz - p_xx*p_yz*p_zy - p_xy*p_yx*p_zz + p_xy*p_yz*p_zx + p_xz*p_yx*p_zy - p_xz*p_yy*p_zx);
  r_xz = (p_xy*p_yz - p_xz*p_yy)/(p_xx*p_yy*p_zz - p_xx*p_yz*p_zy - p_xy*p_yx*p_zz + p_xy*p_yz*p_zx + p_xz*p_yx*p_zy - p_xz*p_yy*p_zx);
  r_xy = (-p_xx*p_xx*p_yy*p_zy*r_xz*r_yy - p_xx*p_xx*p_zy*p_zy*r_xz*r_yz + p_xx*p_xy*p_yx*p_zy*r_xz*r_yy + p_xx*p_xy*p_yy*p_zx*r_xz*r_yy + 2*p_xx*p_xy*p_zx*p_zy*r_xz*r_yz - p_xx*p_xy*p_zz - p_xy*p_xy*p_yx*p_zx*r_xz*r_yy - p_xy*p_xy*p_zx*p_zx*r_xz*r_yz)/(p_xx*p_xx*p_yy*p_yy*r_yy + p_xx*p_xx*p_yy*p_zy*r_yz - 2*p_xx*p_xy*p_yx*p_yy*r_yy - p_xx*p_xy*p_yx*p_zy*r_yz - p_xx*p_xy*p_yy*p_zx*r_yz + p_xy*p_xy*p_yx*p_yx*r_yy + p_xy*p_xy*p_yx*p_zx*r_yz);
  r_xx = -(p_yy*r_xy + p_zy*r_xz)/p_xy;

  test_8 << r_xx, r_xy, r_xz, r_yx, r_yy, r_yz, r_zx, r_zy, r_zz;


  std::cout << "test 1" << std::endl;
  std::cout << test_1 << std::endl;

  std::cout << "test 2" << std::endl;
  std::cout << test_2 << std::endl;

  std::cout << "test 3" << std::endl;
  std::cout << test_3 << std::endl;

  std::cout << "test 4" << std::endl;
  std::cout << test_4 << std::endl;

  std::cout << "test 5" << std::endl;
  std::cout << test_5 << std::endl;

  std::cout << "test 6" << std::endl;
  std::cout << test_6 << std::endl;

  std::cout << "test 7" << std::endl;
  std::cout << test_7 << std::endl;

  std::cout << "test 8" << std::endl;
  std::cout << test_8 << std::endl;

  std::cout << "check 1" << std::endl;
  std::cout << test_1*_rot_z << std::endl;

  std::cout << "check 2" << std::endl;
  std::cout << test_2*_rot_z << std::endl;

  std::cout << "check 3" << std::endl;
  std::cout << test_3*_rot_z << std::endl;

  std::cout << "check 4" << std::endl;
  std::cout << test_4*_rot_z << std::endl;

  std::cout << "check 5" << std::endl;
  std::cout << test_5*_rot_z << std::endl;

  std::cout << "check 6" << std::endl;
  std::cout << test_6*_rot_z << std::endl;

  std::cout << "check 7" << std::endl;
  std::cout << test_7*_rot_z << std::endl;

  std::cout << "check 8" << std::endl;
  std::cout << test_8*_rot_z << std::endl;
   */

 //std::cout << "_rot_z" << std::endl;
 //std::cout << _rot_z << std::endl;
 _rot_z(2,0)  = 0.0;
 _rot_z(2,1)  = 0.0;
 _rot_z(1,1)  = _rot_z(0,0);
 _rot_z(0,1)  = -_rot_z(1,0);
 _rot_z(2,2)  = _rot_z(2,2)/std::fabs(_rot_z(2,2));
 _rot_z(0,2)  = 0.0;
 _rot_z(1,2)  = 0.0;
 /*
 std::cout << "simplified _rot_z" << std::endl;
 std::cout << _rot_z << std::endl;

 */
 _offset_orientation = _rot_z.transpose();
/*
 std::cout << "final _offset_orientation" << std::endl;
 std::cout << _offset_orientation << std::endl;
 std::cout << "check _offset_orientation" << std::endl;
 std::cout << _offset_orientation*_rotation << std::endl;
 */
  //_offset_z = _base.tail<1>()[0];

  _initialized = true;


    return _initialized;
}
