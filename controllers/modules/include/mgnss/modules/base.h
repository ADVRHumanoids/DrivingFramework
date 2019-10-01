#ifndef __MGNSS__CONTROLLERS_BASE_H
#define __MGNSS__CONTROLLERS_BASE_H

#include <mwoibn/robot_class/robot.h>
#include <mwoibn/common/logger.h>

namespace mgnss
{
namespace modules
{



class Base
{

public:
Base(mwoibn::robot_class::Robot& robot) : _robot(robot), kinematics(true), modify(false), dynamics(false){
}

virtual ~Base() {
}

virtual void init() = 0;
virtual void update() = 0;
virtual void send() = 0;
virtual void stop() = 0;
virtual void close() = 0;
virtual void setRate(double rate){
        _robot.setRate(rate);
}

virtual void startLog(mwoibn::common::Logger& logger){
        log(logger, 0);
        logger.start();
}

virtual void log(mwoibn::common::Logger& logger, double time) = 0;

mwoibn::robot_class::Robot& model(){return _robot;};

mwoibn::common::Flag kinematics;
mwoibn::common::Flag modify;
mwoibn::common::Flag dynamics;
const std::string& name(){return _name;}

protected:
mwoibn::robot_class::Robot& _robot;
std::string _name = "";

};
}
}

#endif // WHEELED_MOTION_H
