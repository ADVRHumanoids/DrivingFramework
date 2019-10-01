#ifndef __MWOIBN__COMMUNICATION_MODULES__ROS_SET_LINEAR_POINT_H
#define __MWOIBN__COMMUNICATION_MODULES__ROS_SET_LINEAR_POINT_H

//#include "mwoibn/communication_modules/basic_point.h"
//#include "ros/ros.h"
#include "mwoibn/communication_modules/communication_base.h"
#include <communication_modules_ros_extension/linear_point.h>
#include "mwoibn/robot_points/point.h"
#include "mwoibn/point_handling/state.h"

namespace mwoibn
{
namespace communication_modules
{

  template<typename Publisher, typename Node>
  class RosSetRobotPoint : public CommunicationBase
  {

  public:
    // for now only full robot is supported for this controller
    RosSetRobotPoint(mwoibn::robot_points::Point& point, std::string topic, std::shared_ptr<Node> node)
        : CommunicationBase(3), _point(point), _pub(node)
    {
        _pub.template advertise<communication_modules_ros_extension::linear_point>(topic);

        std::cout << "Loaded ROS linear point publisher in topic " << topic << std::endl;
    }

    RosSetRobotPoint(RosSetRobotPoint& other)
        : CommunicationBase(3), _msg(other._msg), _point(other._point), _pub(other._pub)
    {
      _pub.template reset<communication_modules_ros_extension::linear_point>(other._pub);
    }


    RosSetRobotPoint(RosSetRobotPoint&& other)
    : BasicPoint(other), _msg(other._msg), _point(other._point), _pub(other._pub)
    {
      _pub.template reset<communication_modules_ros_extension::linear_point>(other._pub);
    }


    virtual ~RosSetRobotPoint() {}

    virtual mwoibn::VectorInt map() const {
      return mwoibn::eigen_utils::iota(3);
    }
    //virtual bool initialized() { return _initialized; }
    virtual bool update() { run(); return true;}

    virtual bool run() {

        _msg.x = _point.get()[0];
        _msg.y = _point.get()[1];
        _msg.z = _point.get()[2];

      _initialized = true;
      _pub.template publish<communication_modules_ros_extension::linear_point>(_msg);
      return true;
    }


  protected:
    communication_modules_ros_extension::linear_point _msg;
    Publisher _pub;
    mwoibn::robot_points::Point& _point;



  };


    template<typename Publisher, typename Node>
    class RosSetLinearPoint : public CommunicationBase
    {

    public:
      // for now only full robot is supported for this controller
      RosSetLinearPoint(mwoibn::point_handling::State& point, std::string topic, std::shared_ptr<Node> node)
          : CommunicationBase(3), _point(point), _pub(node)
      {
          _pub.template advertise<communication_modules_ros_extension::linear_point>(topic);

          std::cout << "Loaded ROS linear point publisher in topic " << topic << std::endl;
      }

      RosSetLinearPoint(RosSetLinearPoint& other)
          : CommunicationBase(3), _msg(other._msg), _point(other._point), _pub(other._pub)
      {
        _pub.template reset<communication_modules_ros_extension::linear_point>(other._pub);
      }


      RosSetLinearPoint(RosSetLinearPoint&& other)
      : BasicPoint(other), _msg(other._msg), _point(other._point), _pub(other._pub)
      {
        _pub.template reset<communication_modules_ros_extension::linear_point>(other._pub);
      }


      virtual ~RosSetLinearPoint() {}

      virtual mwoibn::VectorInt map() const {
        return mwoibn::eigen_utils::iota(3);
      }
      //virtual bool initialized() { return _initialized; }
      virtual bool update() { run(); return true;}

      virtual bool run() {
          _state = _point.getWorld();
          _msg.x = _state[0];
          _msg.y = _state[1];
          _msg.z = _state[2];

        _initialized = true;
        _pub.template publish<communication_modules_ros_extension::linear_point>(_msg);
        return true;
      }


    protected:
      communication_modules_ros_extension::linear_point _msg;
      Publisher _pub;
      mwoibn::point_handling::State& _point;
      mwoibn::Vector3 _state;


    };
}
}

#endif // COMMUNICATION_MODULES_ROS_BASIC_H
