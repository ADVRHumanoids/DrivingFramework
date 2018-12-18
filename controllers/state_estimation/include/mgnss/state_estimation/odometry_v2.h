#ifndef __MGNSS_STATE_ESTIMATION__ODOMETRY_V2_H
#define __MGNSS_STATE_ESTIMATION__ODOMETRY_V2_H

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
class OdometryV2 : public mgnss::modules::Base {
public:
// OdometryV2(mwoibn::robot_class::Robot& robot, std::vector<std::string> names, double r);
OdometryV2(mwoibn::robot_class::Robot& robot, std::string config_file, std::string name);
OdometryV2(mwoibn::robot_class::Robot& robot, YAML::Node config);

virtual ~OdometryV2(){
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
virtual void stop(){
}
virtual void close(){
}
virtual void log(mwoibn::common::Logger& logger, double time);

protected:
void _allocate(std::vector<std::string> names);
//mwoibn::robot_class::Robot& _robot; // robot is needed for the joint state and imu feedbacks
void _checkConfig(YAML::Node config);
void _initConfig(YAML::Node config);
mwoibn::VectorN _state, _previous_state, _error, _distance;     // _state - current wheel position


std::unique_ptr<mwoibn::filters::IirSecondOrder> _filter_ptr;

std::vector<mwoibn::point_handling::Point::Current> _estimated, _pelvis, _contact_points, _step;     // _estimated - wheel center position
std::vector<mwoibn::Axis> _directions, _axes;     // directions
double _r;
mwoibn::VectorInt _ids, _base_ids;
mwoibn::Vector6 _base;
mwoibn::Vector3 _base_pos;
mwoibn::Quaternion _imu, _swing, _twist_raw, _twist_es;
mwoibn::VectorN _twists, _sum_twists, __es_2, __st_2;
mwoibn::point_handling::PositionsHandler _wheels_ph;
mwoibn::VectorBool _selector_th, _selector, _contacts;
mwoibn::Axis _x, _y, _z;

mwoibn::VectorInt _base_map;


void _compute1();
void _compute2();
void _compute3();

void _compute1Theta();
void _computeTheta();
void _averageTheta();

void _increment();
void _average();
void _distances();
void _angles();
void _estimateTwist();
void _applyTwist();
void _removeTwist();

void _mad();
void _poseEstimation();
void _filter();
int _max(mwoibn::VectorBool& selector, const mwoibn::VectorN& distance);
int _min(mwoibn::VectorBool& selector, const mwoibn::VectorN& distance);

mwoibn::Vector6 _base_raw, _base_filtered;
//    mwoibn::point_handling::FullStatesHandler _wheels_ph;
//    mwoibn::point_handling::OrientationsHandler _directions_ph;
std::chrono::high_resolution_clock::time_point _begin, _end;



};

}
}
#endif
