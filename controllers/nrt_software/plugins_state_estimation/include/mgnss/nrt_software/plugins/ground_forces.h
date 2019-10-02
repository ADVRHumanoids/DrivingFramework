#ifndef __MGNSS_ROS_PLUGINS__GROUND_FORCES_H
#define __MGNSS_ROS_PLUGINS__GROUND_FORCES_H

#include "mgnss/plugins/generator.h"
#include "mgnss/state_estimation/ground_forces.h"
#include "mwoibn/communication_modules/ros_point_feeback.h"
#include "mwoibn/communication_modules/shared_point_setter.h"
#include "mwoibn/communication_modules/ros_set_linear_point.h"

#include "mgnss/ros_callbacks/ground_forces.h"

#include "geometry_msgs/WrenchStamped.h"


namespace mgnss
{
namespace nrt_software {
namespace plugins
{
  // generator specialization for the ground forces estimation module
  template<typename Subscriber, typename Service, typename Node, typename Publisher>
  class GroundForces : public mgnss::plugins::Generator<Subscriber, Service, Node, Publisher>
  {
    typedef mgnss::plugins::Generator<Subscriber, Service, Node, Publisher> Generator_;


public:
GroundForces() : Generator_("ground_forces"){

}

virtual ~GroundForces(){
}

protected:

//! Function that creates an instance of the ground forces estimation module
virtual void _resetPrt(YAML::Node config){
        Generator_::controller_ptr.reset(new mgnss::state_estimation::GroundForces(*Generator_::_robot_ptr.begin()->second, config));
}


//! Generate the communication layer
virtual void _initCallbacks(YAML::Node config){

  // Publish the estimated center of pressure to the ROS topic 
  Generator_::_robot_ptr.begin()->second->controllers.add( mwoibn::communication_modules::RosSetRobotPoint<Publisher, Node>(Generator_::_robot_ptr.begin()->second->centerOfPressure(), "center_of_pressure", Generator_::n), "centerOfPressure");
  // read the contact configuration file
  if(!config["contact_source"])
    throw(std::invalid_argument(__PRETTY_FUNCTION__ + std::string(" Could not find a contact source.")));

  YAML::Node contacts =  mwoibn::robot_class::Robot::getConfig(mwoibn::robot_class::Robot::readPath(config["contact_source"]));

  for(auto entry: contacts)
      std::cout << "YAML " << entry.first << std::endl;

  // Publish the estimated contact point force to the ROS topic
  for(auto& contact: Generator_::_robot_ptr.begin()->second->contacts()){
      std::cout << "contact " << contact->getName() << std::endl;
      Generator_::_robot_ptr.begin()->second->controllers.add(mwoibn::communication_modules::RosSetLinearPoint<Publisher, Node>(contact->wrench().force, contact->getName(), Generator_::n), "ros_"+contact->getName());
  }


}

//! //! Generate the communication into the shared space
virtual void _initCallbacks(YAML::Node config, mwoibn::communication_modules::Shared& share){

  // read the contact configuration file
  if(!config["contact_source"])
    throw(std::invalid_argument(__PRETTY_FUNCTION__ + std::string(" Could not find a contact source.")));

  std::string prefix = config["prefix"] ? config["prefix"].template as<std::string>() : "";
  std::string file = mwoibn::robot_class::Robot::readPath(config["contact_source"]);
  YAML::Node contacts;

  try
  {
          contacts = YAML::LoadFile(file);
  }
  catch (const YAML::BadFile& e)
  {
          throw std::invalid_argument(
                        std::string("Could not find the configuration file: ") + file);
  }
  catch (...)
  {
          throw;
  }

  // read the configuration file for the contact points
   YAML::Node loaded;
  for(auto entry: contacts[Generator_::_robot_ptr.begin()->second->name()]["contacts"]){
      loaded[entry.second["name"].template as<std::string>()] = entry.second;
      loaded[entry.second["name"].template as<std::string>()]["name"]  = prefix + loaded[entry.second["name"].template as<std::string>()]["name"].template as<std::string>();
  }


  // Create the publisher of the estimated contact forces in the shared space
  for(auto& contact: Generator_::_robot_ptr.begin()->second->contacts())
      Generator_::_robot_ptr.begin()->second->controllers.add(mwoibn::communication_modules::SharedPointSetter(
                              loaded[contact->getName()], share, contact->wrench()), prefix+contact->getName());


}


};
}
}
}
#endif // RT_MY_TEST_H
