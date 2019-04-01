#ifndef __MGNSS_STATE_ESTIMATION__ODOMETRY_V3_H
#define __MGNSS_STATE_ESTIMATION__ODOMETRY_V3_H

//#include <mwoibn/robot_class/robot.h>
#include "mgnss/modules/base.h"
#include <mwoibn/point_handling/robot_points_handler.h>
#include <mwoibn/point_handling/spatial_velocity.h>
#include <mwoibn/point_handling/handler.h>
#include <mwoibn/point_handling/frame_plus.h>

#include <mwoibn/filters/iir_second_order.h>
#include <chrono>

namespace mgnss {

namespace state_estimation {

template<typename Type>
class Estimaton{

public:

  virtual void update() = 0;
  const Type & get() {return _estimate;}

protected:
  std::vector<Type> _estimates;
  std::vector<double> _measures;
  mwoibn::VectorBool _selector;
  Type _estimate;


  int _min(){
            int id = -1;
            double value = mwoibn::MAX_DOUBLE;
            for (int i = 0; i < _selector.size(); i++)
            {
                    if (_selector[i] && _measures[i] < value)
                    {
                            id = i;
                            value = _measures[i];
                    }
            }
            return id;
  }

  int _max()
  {

          int id = -1;
          double value = -1;
          for (int i = 0; i < _selector.size(); i++)
          {
                  if (_selector[i] && _measures[i] > value)
                  {
                          id = i;
                          value = _measures[i];
                  }
          }

          return id;
  }



  virtual void _average() = 0;
  virtual void _assign()
  {
        _measure();
        int m = _min();
        _estimate = _estimates[m];
  }

  virtual void _measure() = 0;
  virtual void _compute(){
    if (_selector.count() < 3)
    {
            _average(); // return average no better option?
            return;
    }

    if (_selector.count() == 3)
    {
            _assign();
            return;
    }

    _measure();
    _selector[_max()] = 0;

    _compute();
  }


};

class Twist: public Estimaton<double>{

public:
  Twist(int size, std::vector<mwoibn::point_handling::Point::Current>& contact_points,
                  std::vector<mwoibn::point_handling::Point::Current>& step,
                  std::vector<mwoibn::point_handling::Point::Current>& base,
                  const mwoibn::VectorBool& leg_selector): _contact_points(contact_points), _step(step), _base(base), _leg_selector(leg_selector)
  {
    _estimates.assign(mwoibn::std_utils::factorial(size)/mwoibn::std_utils::factorial(2)/mwoibn::std_utils::factorial(size-2), 0);
    _measures.assign(mwoibn::std_utils::factorial(size)/mwoibn::std_utils::factorial(2)/mwoibn::std_utils::factorial(size-2), mwoibn::MAX_DOUBLE);
    _selector.setZero(mwoibn::std_utils::factorial(size)/mwoibn::std_utils::factorial(2)/mwoibn::std_utils::factorial(size-2));
    __es_2.setZero(2);
    __st_2.setZero(2);

  }

  void update();

protected:
  mwoibn::VectorN __es_2, __st_2;
  std::vector<mwoibn::point_handling::Point::Current>& _contact_points;
  std::vector<mwoibn::point_handling::Point::Current>& _step;
  std::vector<mwoibn::point_handling::Point::Current>& _base;
  const mwoibn::VectorBool& _leg_selector;

  virtual void _average();

  virtual void _measure();

};

class Velocity: public Estimaton<mwoibn::VectorN>{

public:
  Velocity(int size, const mwoibn::VectorBool& leg_selector, mwoibn::point_handling::Handler<mwoibn::point_handling::SpatialVelocity>& wheels_velocity,
    const mwoibn::Axis&  ground_normal, double r, const mwoibn::VectorInt& ids, mwoibn::Quaternion& twist_es):
    _leg_selector(leg_selector), _wheels_velocity(wheels_velocity), _ground_normal(ground_normal), _r(r), _ids(ids), _twist_es(twist_es)
  {
    _estimates.assign(mwoibn::std_utils::factorial(size)/mwoibn::std_utils::factorial(2)/mwoibn::std_utils::factorial(size-2), mwoibn::VectorN::Zero(4));
    _measures.assign(mwoibn::std_utils::factorial(size)/mwoibn::std_utils::factorial(2)/mwoibn::std_utils::factorial(size-2), mwoibn::MAX_DOUBLE);
    _selector.setZero(mwoibn::std_utils::factorial(size)/mwoibn::std_utils::factorial(2)/mwoibn::std_utils::factorial(size-2));
    _inverse.reset(new mwoibn::PseudoInverse(mwoibn::Matrix(6,4 )));
    _estimate.setZero(4);
  }

  void update();

protected:;
  const mwoibn::VectorBool& _leg_selector;
  std::unique_ptr<mwoibn::PseudoInverse> _inverse;
  mwoibn::point_handling::Handler<mwoibn::point_handling::SpatialVelocity>& _wheels_velocity;
  const mwoibn::Axis&  _ground_normal;
  const mwoibn::VectorInt& _ids;
  mwoibn::Quaternion& _twist_es;
  double _r;

  virtual void _average();

  virtual void _measure();
  void _update(int n);


};


/**
 * it assumes the pelvis orientation is provided by the external source? - in my case the imu? - do it using the online floating base feedback - remove the postion estimation
 */
class OdometryV3 : public mgnss::modules::Base {
public:
// OdometryV3(mwoibn::robot_class::Robot& robot, std::vector<std::string> names, double r);
OdometryV3(mwoibn::robot_class::Robot& robot, std::string config_file, std::string name);
OdometryV3(mwoibn::robot_class::Robot& robot, YAML::Node config);

virtual ~OdometryV3(){
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
std::unique_ptr<mwoibn::filters::IirSecondOrder> _vel_ptr;

mwoibn::point_handling::Handler<mwoibn::point_handling::SpatialVelocity> _wheels_velocity;
mwoibn::point_handling::Handler<mwoibn::point_handling::FramePlus> _wheels_frames;

std::vector<mwoibn::point_handling::Point::Current> _estimated, _pelvis, _contact_points, _step;     // _estimated - wheel center position
std::vector<mwoibn::Axis> _directions, _axes;     // directions
double _r;
mwoibn::VectorInt _ids, _base_ids;
mwoibn::Vector6 _base;
mwoibn::Vector3 _base_pos, _ang_vel, _temp_est, _temp_fil;
mwoibn::Quaternion _imu, _last_imu, _swing, _twist_raw, _twist_es;

mwoibn::VectorBool _selector_th, _selector, _contacts;
mwoibn::Axis _x, _y, _z, _ground_normal;

mwoibn::Matrix3 _projection, _to_euler;
mwoibn::Vector3 _euler_vel;
mwoibn::VectorInt _base_map;

std::unique_ptr<Twist> _twist_ptr;
std::unique_ptr<Velocity> _velocity_ptr;


void _compute1();
void _compute2();
void _compute3();

void _computeVel();
void _compute1Vel();

void _averageVel();

void _increment();
void _average();
void _distances();
void _angles();

void _applyTwist();
void _removeTwist();

const mwoibn::VectorN& _estimateVelocity(int n);

int _max(mwoibn::VectorBool& selector, const mwoibn::VectorN& distance);
int _min(mwoibn::VectorBool& selector, const mwoibn::VectorN& distance);

void _mad();
void _poseEstimation();
void _filter();

mwoibn::Vector6 _base_raw, _base_filtered;

std::chrono::high_resolution_clock::time_point _begin, _end;



};

}
}
#endif
