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
  template<typename Subscriber, typename Service, typename Node, typename Publisher>
  class GroundForces : public mgnss::plugins::Generator<Subscriber, Service, Node, Publisher>
  {
    typedef mgnss::plugins::Generator<Subscriber, Service, Node, Publisher> Generator_;


public:
// GroundForces(int argc, char** argv) : mgnss::plugins::RosBase(argc, argv, "ground_forces"){
// }
GroundForces() : Generator_("ground_forces"){

}

virtual ~GroundForces(){
}

protected:

virtual void _resetPrt(YAML::Node config){
        Generator_::controller_ptr.reset(new mgnss::state_estimation::GroundForces(*Generator_::_robot_ptr.begin()->second, config));
}


virtual void _initCallbacks(YAML::Node config){

  Generator_::_robot_ptr.begin()->second->controllers.add( mwoibn::communication_modules::RosSetRobotPoint<Publisher, Node>(Generator_::_robot_ptr.begin()->second->centerOfPressure(), "center_of_pressure", Generator_::n), "centerOfPressure");
  if(!config["contact_source"])
    throw(std::invalid_argument(__PRETTY_FUNCTION__ + std::string(" Could not find a contact source.")));

  YAML::Node contacts =  mwoibn::robot_class::Robot::getConfig(mwoibn::robot_class::Robot::readPath(config["contact_source"]));

  for(auto entry: contacts)
      std::cout << "YAML " << entry.first << std::endl;

  for(auto& contact: Generator_::_robot_ptr.begin()->second->contacts()){
      std::cout << "contact " << contact->getName() << std::endl;
      Generator_::_robot_ptr.begin()->second->controllers.add(mwoibn::communication_modules::RosSetLinearPoint<Publisher, Node>(contact->wrench().force, contact->getName(), Generator_::n), "ros_"+contact->getName());
        //mwoibn::communication_modules::RosPointSet(, contacts[contact.getName()]));
  }

  // Generator_::_sub.push_back(Generator_::n->template subscribe<geometry_msgs::WrenchStamped>("/centauro/j_ft_pelvis_ft_sensor", 1, boost::bind(&mgnss::ros_callbacks::ground_forces::wrench,
  //   _1, static_cast<mgnss::state_estimation::GroundForces*>(Generator_::controller_ptr.get()))));

}

virtual void _initCallbacks(YAML::Node config, mwoibn::communication_modules::Shared& share){

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
                        std::string("Couldn\t find the configuration file: ") + file);
  }
  catch (...)
  {
          throw;
  }

   YAML::Node loaded;
  for(auto entry: contacts[Generator_::_robot_ptr.begin()->second->name()]["contacts"]){
      loaded[entry.second["name"].template as<std::string>()] = entry.second;
      loaded[entry.second["name"].template as<std::string>()]["name"]  = prefix + loaded[entry.second["name"].template as<std::string>()]["name"].template as<std::string>();
  }


  for(auto& contact: Generator_::_robot_ptr.begin()->second->contacts())
      Generator_::_robot_ptr.begin()->second->controllers.add(mwoibn::communication_modules::SharedPointSetter(
                              loaded[contact->getName()], share, contact->wrench()), prefix+contact->getName());
  //
  // Generator_::_sub.push_back(Generator_::n->template subscribe<geometry_msgs::WrenchStamped>("/centauro/j_ft_pelvis_ft_sensor", 1, boost::bind(&mgnss::ros_callbacks::ground_forces::wrench,
  //       _1, static_cast<mgnss::state_estimation::GroundForces*>(Generator_::controller_ptr.get()))));

  //_initCallbacks(config);

}


};
}
}
}
#endif // RT_MY_TEST_H