#ifndef __MGNSS_STATE_ESTIMATION__ACCELERATION_TEST_H
#define __MGNSS_STATE_ESTIMATION__ACCELERATION_TEST_H

#include <mwoibn/robot_class/robot.h>
#include <mwoibn/point_handling/linear_acceleration.h>
#include <mwoibn/point_handling/frame_plus.h>
#include <mwoibn/filters/iir_second_order.h>



#include "mgnss/modules/base.h"

namespace mgnss {

namespace state_estimation {

/**
 * it assumes the pelvis orientation is provided by the external source? - in my case the imu? - do it using the online floating base feedback - remove the postion estimation
 */
class AccelerationTest : public mgnss::modules::Base {
public:
AccelerationTest(mwoibn::robot_class::Robot& robot);
AccelerationTest(mwoibn::robot_class::Robot& robot, std::string config_file, std::string name);
AccelerationTest(mwoibn::robot_class::Robot& robot, YAML::Node config);

virtual ~AccelerationTest(){
}

virtual void update();
virtual void init();

virtual void send(){
        //_robot.send();
}

virtual void stop(){}

virtual void close(){}

virtual void log(mwoibn::common::Logger& logger, double time);

protected:
void _allocate();
void _checkConfig(YAML::Node config);
void _initConfig(YAML::Node config);

mwoibn::point_handling::FramePlus _frame;

mwoibn::point_handling::LinearAcceleration _acceleration;
mwoibn::VectorN _vel_p, _acc_est, _point_acc;//, _f_acc_est;


std::unique_ptr<mwoibn::filters::IirSecondOrder> _filter_ptr;

};

}
}
#endif
