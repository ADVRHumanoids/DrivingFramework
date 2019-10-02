#ifndef __MGNSS__PLUGINS__ROS_BASE_H
#define __MGNSS__PLUGINS__ROS_BASE_H

#include "mgnss/plugins/generator.h"
#include <mwoibn/common/ros_logger.h>
#include <ros/ros.h>
#include "mwoibn/std_utils/map.h"


namespace mgnss::plugins
{
//! Wrapper over the ros publisher to provide a common interface in ROS and XBotCore middlewares. Ros implementations.
class RosPublisher{

public:
  RosPublisher(std::shared_ptr<ros::NodeHandle> node){
    _node.reset(new ros::NodeHandle());
  }
  //! Destructor
  /* shutdowns the publisher */
  ~RosPublisher(){
    _pub.shutdown();
  }

  /* Function to copy the publisher */
  template<typename Message>
  void reset(const RosPublisher& other){
      std::string topic = other._pub.getTopic();
      _pub = _node->advertise<Message>(topic, 1);
  }

  /* Function to move the publisher */
  template<typename Message>
  void reset(RosPublisher&& other){
      std::string topic = other._pub.getTopic();
      other._pub.shutdown();
      _pub = _node->advertise<Message>(topic, 1);
  }

  //! Publish the message
  template<typename Message>
  void publish(const Message& msg ){
    _pub.publish(msg);
  }

  //! Advertise the publisher
  template<typename Message>
 void advertise(const std::string& topic){
   _pub = _node->advertise<Message>(topic, 1);
 }

protected:
  std::shared_ptr<ros::NodeHandle> _node;
  ros::Publisher _pub;
};
}


//! C macro to keep readable name to dynamically load plugins
#define MGNSS_REGISTER_ROS_PLUGIN_(constructor) \
extern "C" mgnss::plugins::RosBase* make() \
{ \
  return new mgnss::plugins::RosPlugin<constructor<ros::Subscriber, ros::ServiceServer, ros::NodeHandle, mgnss::plugins::RosPublisher>>(); \
} \

namespace mgnss
{
namespace plugins
{

//! A class that directly provides the ros plugins. It contains the Generator specialized for ros
class RosBase
{

  typedef  std::map<std::string, std::shared_ptr<mwoibn::robot_class::Robot>> robot_map;
  // convinient variable that defines the specialization of the generator template
  typedef  mgnss::plugins::Generator<ros::Subscriber, ros::ServiceServer, ros::NodeHandle, RosPublisher> RosGenerator_;
public:

RosBase() { }

//! establish a connection with ROS
virtual void connect(int argc, char** argv, std::string name);
//! establish a connection with ROS
virtual void connect(std::string name);

virtual ~RosBase(){
}

//! creates the Generator plugin, robot models and logger
virtual bool init();
//! creates the Generator plugin, robot models and logger in the shared space
virtual bool init(robot_map& share_robots, std::shared_ptr<mwoibn::common::Logger>& logger_ptr, std::shared_ptr<ros::NodeHandle> n, mwoibn::communication_modules::Shared& share, std::string name);

//! executed when the plugin is shut down. It closes the plugin and saves the logged data
virtual bool close(){ _plugin_ptr->close(); return true;}

//! executed when the plugin is turned on
/** Checks the validity of all the feedbacks and sets initial conditions */
virtual void start(double time){_plugin_ptr->start(time);}

//! executed when the plugin has been stoped
virtual void stop(){_plugin_ptr->stop();}
//! executed at each control step
virtual void control_loop(double time){_plugin_ptr->control_loop(time);}

// returns the reference to the instance of the specialized Generator
  RosGenerator_& plugin(){return *_plugin_ptr;}

protected:
std::unique_ptr<RosGenerator_> _plugin_ptr; // instance of the Generator template specialized for ROS

//! Read a path to the config file from the ROS parameter server
std::string _configFile();


};

//! Template over the RosBase to dynamically load the requested plugin
template<typename Plugin>
class RosPlugin: public RosBase{
public:
    RosPlugin(): RosBase(){
        _plugin_ptr.reset(new Plugin());
    }
};

}
}
#endif // __MGNSS__PLUGINS__ROS_BASE_H
