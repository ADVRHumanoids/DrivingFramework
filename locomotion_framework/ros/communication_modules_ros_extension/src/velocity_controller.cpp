#include "mwoibn/communication_modules/velocity_controller.h"

bool mwoibn::communication_modules::VelocityController::run()
{

  if (_velocity)
  {
    mapTo(_command.velocity.get(),
                    _des_q.velocity);
  }


  _command_pub.publish(_des_q);

  return true;
}


bool mwoibn::communication_modules::VelocityController::loadGains(custom_services::loadGains::Request& req, custom_services::loadGains::Response& res){
    YAML::Node config;
    try
    {
            config = YAML::LoadFile(req.file);
    }
    catch (const YAML::BadFile& e)
    {
            res.message = "Couldn\t find the configuration file: " + req.file;
            res.success = false;
            return false;
    }
    catch (...)
    {
      res.message = "Unkown error reading file: " + req.file;
      res.success = false;
      return false;
    }

    std::cout << _name << std::endl;
    //std::cout << _node.getNamespace().substr(1) << std::endl;

    if(!config[_node.getNamespace().substr(1)]) {
      res.message = "Couldn't find robot configuration in the config file: " + req.file + "\t" + _node.getNamespace().substr(1);
      res.success = false;
      return false;
    }

    config = config[_node.getNamespace().substr(1)];

    if(!config[_name]) {
      res.message = "Couldn't find controller configuration in the config file: " + req.file + "\t" + _name;
      res.success = false;
      return false;
    }

    config = config[_name];

    ros::ServiceClient names = _node.serviceClient<custom_services::getJointNames>(_name + "/get_joint_names");
    ros::ServiceClient gains = _node.serviceClient<custom_services::updateGains>(_name + "/update_gains");


    custom_services::getJointNames srv;

    if (!names.call(srv))
    {
      std::cout << "WARNING: Couldn't retrive joint names from the controller:\n\t " << _name << "/get_joint_names" << std::endl;
    }

    custom_services::updateGains gains_srv;
    for (auto id: srv.response.ids){
        gains_srv.request.nr = id;
        gains_srv.request.a_p = config["joint"+std::to_string(id+1)]["a_Kp"].as<double>();
        gains_srv.request.a_d = config["joint"+std::to_string(id+1)]["a_Kd"].as<double>();
        gains_srv.request.j_p = config["joint"+std::to_string(id+1)]["j_Kp"].as<double>();
        gains_srv.request.j_d = config["joint"+std::to_string(id+1)]["j_Kd"].as<double>();
        gains_srv.request.i = config["joint"+std::to_string(id+1)]["Ki"].as<double>();

        if(!gains.call(gains_srv))
          std::cout << "Couldn't change gains for " << config["joint"+std::to_string(id+1)]["name"] << std::endl;
        else
          std::cout << gains_srv.response << std::endl;
        std::cout << id << "\t" << config["joint"+std::to_string(id+1)]["name"] << std::endl;
      }

    res.success = true;

    return true;
}
