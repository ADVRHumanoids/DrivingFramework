#ifndef __MGNSS_ROS_PLUGINS__GROUND_FORCES_H
#define __MGNSS_ROS_PLUGINS__GROUND_FORCES_H

#include "mgnss/plugins/generator.h"
#include "mgnss/state_estimation/ground_forces.h"
#include "mwoibn/communication_modules/ros_point_feeback.h"
#include "mwoibn/communication_modules/shared_point_setter.h"

namespace mgnss
{
namespace nrt_software {
namespace plugins
{
  template<typename Subscriber, typename Service, typename Node>
  class GroundForces : public mgnss::plugins::Generator<Subscriber, Service, Node>
  {
    typedef mgnss::plugins::Generator<Subscriber, Service, Node> Generator_;


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

  if(!config["contact_source"])
    throw(std::invalid_argument(__PRETTY_FUNCTION__ + std::string(" Could not find a contact source.")));

  YAML::Node contacts =  mwoibn::robot_class::Robot::getConfig(mwoibn::robot_class::Robot::readPath(config["contact_source"]));

  for(auto entry: contacts)
      std::cout << "YAML " << entry.first << std::endl;

  for(auto& contact: Generator_::_robot_ptr.begin()->second->contacts()){
      std::cout << "contact " << contact->getName() << std::endl;
          //_robot_ptr.begin()->second->controllers.add( mwoibn::communication_modules::RosPointSet(contact, contacts[contact.getName()]));
  }
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
}


};
}
}
}
#endif // RT_MY_TEST_H
