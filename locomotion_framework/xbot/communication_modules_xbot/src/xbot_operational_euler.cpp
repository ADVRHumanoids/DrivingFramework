#include "mwoibn/communication_modules/xbot_operational_euler.h"

mwoibn::communication_modules::XBotOperationalEuler::XBotOperationalEuler(
    mwoibn::robot_class::State& command, mwoibn::robot_class::BiMap map,
    YAML::Node config, XBot::RobotInterface& robot, double rate)
    : BasicOperationalEuler(command, map, config), _rate(rate)
{

  std::map<std::string, XBot::ImuSensor::ConstPtr> imus = robot.getImu();

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

//  std::cout << "offset\n" << _offset_position << std::endl;

//  _linear_state << 0,0,0; // start from the zero position?
//  _rotation << 1,0,0,0,1,0,0,0,1;
  
//  BasicOperationalEuler::getPosition(_rotation, _linear_state);
    
  
//  std::cout << "Loaded xbot operational feedback " << config["name"] << std::endl;

//  std::cout <<_map_dofs << std::endl;

}

bool mwoibn::communication_modules::XBotOperationalEuler::get()
{

  _imu->getOrientation(_rotation);

  _command.get(_base, _map_dofs, mwoibn::robot_class::INTERFACE::VELOCITY);
//  std::cout << "velocity\n" << _base << std::endl;
  getPosition(_rotation, _base.head(3));
  return true;

}

void mwoibn::communication_modules::XBotOperationalEuler::getPosition(mwoibn::Matrix3 orientation,
                           mwoibn::Vector3 velocity)
  {

    _command.get(_base, _map_dofs, mwoibn::robot_class::INTERFACE::POSITION);
    
    
    _base.head(3) += velocity*_rate;

    //    _base[2] += velocity[2]*_rate;
    _base.tail(3) =
        (_offset_orientation * orientation)
            .eulerAngles(_angels[0], _angels[1],
                         _angels[2]); // Check if the convention is met here


    // std::cout << "after\n" << _base << std::endl;

    _command.set(_base, _map_dofs, mwoibn::robot_class::INTERFACE::POSITION);
  }
