#include "mwoibn/communication_modules/xbot_operational_euler.h"

mwoibn::communication_modules::XBotOperationalEuler::XBotOperationalEuler(
    mwoibn::robot_class::State& command, mwoibn::robot_class::BiMap map,
    YAML::Node config, XBot::RobotInterface& robot)
    : BasicOperationalEuler(command, map, config)
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

  _linear_state << 0,0,0; // the postition estimation is not supported yet
  std::cout << "Loaded xbot operational feedback " << config["name"] << std::endl;


}

bool mwoibn::communication_modules::XBotOperationalEuler::get()
{

  _imu->getOrientation(_rotation);
  getPosition(_rotation, _linear_state);
  return true;

}
