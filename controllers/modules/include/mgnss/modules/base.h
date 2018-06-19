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
Base(mwoibn::robot_class::Robot& robot) : _robot(robot){
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
virtual void startLog(mwoibn::common::Logger& logger) = 0;
virtual void log(mwoibn::common::Logger& logger, double time) = 0;
protected:
mwoibn::robot_class::Robot& _robot;
};
}
}

#endif // WHEELED_MOTION_H
