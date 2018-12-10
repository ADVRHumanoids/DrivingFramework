#ifndef __MGNSS_STATE_ESTIMATION_ODOMETRY_H
#define __MGNSS_STATE_ESTIMATION_ODOMETRY_H

//#include <mwoibn/robot_class/robot.h>
#include "mgnss/modules/base.h"
#include <mwoibn/point_handling/robot_points_handler.h>
#include <mwoibn/filters/iir_second_order.h>
#include <chrono>

namespace mgnss {

namespace state_estimation {

/**
 * it assumes the pelvis orientation is provided by the external source? - in my case the imu? - do it using the online floating base feedback - remove the postion estimation
 */
class Odometry : public mgnss::modules::Base {
public:
Odometry(mwoibn::robot_class::Robot& robot, std::string config_file, std::string name);
Odometry(mwoibn::robot_class::Robot& robot, YAML::Node config);

virtual ~Odometry(){
}

virtual void setRate(double rate){
        mgnss::modules::Base::setRate(rate);
        _filter_ptr->computeCoeffs(_robot.rate());
}

const mwoibn::Vector6& getRaw(){
        return _base_raw;
}
const mwoibn::Vector6& getFiltered(){
        return _base_filtered;
}
const mwoibn::point_handling::Point::Current& getContact(int i){
        return _contact_points[i];
}

const mwoibn::Vector6& get(){
        return _base;
}
virtual void update();
virtual void init();
virtual void send(){
        _robot.send();
}
virtual void stop(){}
virtual void close(){}

virtual void initLog(mwoibn::common::Logger& logger);
virtual void log(mwoibn::common::Logger& logger, double time);

protected:
void _allocate(std::vector<std::string> names);
//mwoibn::robot_class::Robot& _robot; // robot is needed for the joint state and imu feedbacks
void _checkConfig(YAML::Node config);
void _initConfig(YAML::Node config);
mwoibn::VectorN _state, _previous_state, _error, _distance;     // _state - current wheel position
mwoibn::VectorInt _selector, _contacts;     // _state - current wheel position

std::unique_ptr<mwoibn::filters::IirSecondOrder> _filter_ptr;

std::vector<mwoibn::point_handling::Point::Current> _estimated, _pelvis, _contact_points;     // _estimated - wheel center position
std::vector<mwoibn::Vector3> _directions, _axes;     // directions
double _r;
mwoibn::VectorInt _ids;
mwoibn::Vector6 _base;
mwoibn::Vector3 _base_pos;

mwoibn::point_handling::PositionsHandler _wheels_ph;

void _compute1();
void _compute2();
void _compute3();

void _average();
void _distances();

void _mad();

int _max();
int _min();

mwoibn::Vector6 _base_raw, _base_filtered;
//    mwoibn::point_handling::FullStatesHandler _wheels_ph;
//    mwoibn::point_handling::OrientationsHandler _directions_ph;
std::chrono::high_resolution_clock::time_point _begin, _end;



};

}
}
#endif
