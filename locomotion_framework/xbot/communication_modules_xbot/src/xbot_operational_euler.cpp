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
                              "loaded.\n"));  // if there is only one
                                              // sensor this could be
                                              // automatic
        else if (imus.size() == 0)
                throw(std::invalid_argument(
                              "Couldn't find any imu sensor."));  // if there is only one
                                                                  // sensor this could be
                                                                  // automatic
        else
                _imu = imus[config["sensor"].as<std::string>()];

        //  _imu = _robot->getImu(config["name"].as<std::string>());

        if (!_imu)
                throw(std::invalid_argument(
                              "Couldn't find an IMU sensor named " +
                              config["sensor"].as<std::string>()));  // if there is only one sensor this
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

void mwoibn::communication_modules::XBotOperationalEuler::getPosition(mwoibn::Matrix3 orientation, mwoibn::Vector3 position)
{

        if(!_initialized) reset();

        _base.tail(3) =
                (_offset_orientation*orientation)
                .eulerAngles(_angels[0], _angels[1],
                             _angels[2]); // Check if the convention is met here

        //_base.tail<1>()[0] -= _offset_z;
        _command.position.set(_base, _map_dofs);
}

bool mwoibn::communication_modules::XBotOperationalEuler::reset(){

        if(!_size) {
                _initialized = true;
                return _initialized;
        }


        _rot_z = _offset_org*_rotation;

        _rot_z(2,0)  = 0;
        _rot_z(2,1)  = 0;
        _rot_z(1,1)  = _rot_z(0,0);
        _rot_z(0,1)  = -_rot_z(1,0);
        _rot_z(2,2)  = _rot_z(2,2)/std::fabs(_rot_z(2,2));
        _rot_z(0,2)  = 0;
        _rot_z(1,2)  = 0;

        _offset_orientation = _rot_z.transpose();

        _initialized = true;


        return _initialized;
}
