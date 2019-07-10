#include "mgnss/plugins/xbot_shared.h"

REGISTER_XBOT_PLUGIN_( mgnss::plugins::XbotShared ) \

void mgnss::plugins::XbotShared::connect(XBot::Handle::Ptr handle)
{
        _name = "shared";
        _n = handle->getRosHandle();
}

void mgnss::plugins::XbotShared::_checkConfig(YAML::Node plugin_config, std::string config_file)
{
        if (!plugin_config["modules"])
                throw std::invalid_argument(config_file +
                                            std::string("\t Could not find modules configuration."));
        if (!plugin_config["modules"][_name])
                throw std::invalid_argument(config_file +
                                            std::string("\t Could not find ") + _name + std::string(" module configuration."));

        plugin_config = plugin_config["modules"][_name];

        if (!plugin_config["plugins"])
                throw std::invalid_argument(config_file + std::string("\nmodule:\t") + _name +
                                            std::string("\nWhat: Could not find required argument 'plugins'"));

}


void mgnss::plugins::XbotShared::_initModules(XBot::Handle::Ptr handle, YAML::Node plugin_config){
      std::string lib, name, full_arg;

      for(auto plugin: plugin_config){
           full_arg = plugin.as<std::string>();
           size_t pos = full_arg.find("::");
           if ( pos == std::string::npos) {
             lib = full_arg;
             name = full_arg;
            }
           else{
             lib = full_arg.substr(0, pos);
             name = full_arg.substr(pos+2);
           }

          mgnss::plugins::XbotBaseUnify* temp = mgnss::plugins::xbot_make(lib);
          temp->init_control_plugin(handle, _robot_ptr, _logger_ptr, _n, _shared, name);
         _controller_ptrs.push_back(std::move(temp->plugin().releaseController()));
          std::cout << "XbotShared: loaded " << plugin.as<std::string>() << std::endl;
      }
}

std::string mgnss::plugins::XbotShared::_loadConfig(XBot::Handle::Ptr handle, YAML::Node& config, YAML::Node& plugin_config)
{
        // std::string config_file;
        YAML::Node xbot_config = mwoibn::robot_class::Robot::getConfig(handle->getPathToConfigFile());

        // Read path to mwoibn config file
        if (!xbot_config["config_file"])
                throw std::invalid_argument(handle->getPathToConfigFile() +
                                            std::string("\t Please specify MWOIBN config file."));

        std::string config_file = xbot_config["config_file"].as<std::string>();

        // Read MWOIBN config file
        config = mwoibn::robot_class::Robot::getConfig(config_file);

        plugin_config = mwoibn::robot_class::Robot::getConfig(config_file);

        return config_file;
}


bool mgnss::plugins::XbotShared::init_control_plugin(XBot::Handle::Ptr handle)
{
        YAML::Node config, plugin_config;

        connect(handle);
        std::string config_file = _loadConfig(handle, config, plugin_config);

        _checkConfig(plugin_config, config_file);

        _logger_ptr.reset(new mwoibn::common::XbotLogger(_name));
        _initModules(handle, plugin_config["plugins"]);
        std::cout << "XbotShared: loaded " << _robot_ptr.size() << " robot abstractions." << std::endl;
        for(auto& robot: _robot_ptr)
          std::cout << "\t" << robot.first << std::endl;


        _logger_ptr->start();

        return true;
}

void mgnss::plugins::XbotShared::on_start(double time)
{
        _start = time;
        _rate = true;

        _init(time);
}

void mgnss::plugins::XbotShared::_init(double time){

  for(auto& controller: _controller_ptrs){
    if(!controller->model().get()) return;
    if(controller->kinematics.get()) controller->model().updateKinematics();

    controller->init();
            //controller->update();
    controller->send();
    if(controller->modify.get()) controller->model().kinematics_update.set(true);

    _logger_ptr->prefix(controller->name());
    controller->log(*_logger_ptr.get(), time-_start);
  }

  _logger_ptr->write();
  _resetUpdates();
  if(_rate)  _initialized = true;

}

void mgnss::plugins::XbotShared::_resetUpdates(){
      _robot_ptr.begin()->second->wait(true);

  // std::cout << "reset kinematics" << std::endl;
  for(auto& robot: _robot_ptr)
    robot.second->kinematics_update.set(false);

  for(auto& controller: _controller_ptrs){
    if(controller->kinematics.get()) controller->model().kinematics_update.set(true);
  }
}

void mgnss::plugins::XbotShared::on_stop(double time) {
        for(auto& controller: _controller_ptrs)  controller->stop();
}

void mgnss::plugins::XbotShared::control_loop(double time)
{


  if (!_initialized) { _init(time); return; }

  // std::cout << "CONTROLER LOOP" << std::endl;

  for(auto& controller: _controller_ptrs){

    if(!controller->model().get()) return;

    if(controller->kinematics.get())
        controller->model().updateKinematics();
    

    controller->update();
    controller->send();
    if(controller->modify.get()) controller->model().kinematics_update.set(true);
    _logger_ptr->prefix(controller->name());
    controller->log(*_logger_ptr.get(), time-_start);
  }

  _logger_ptr->write();

  _resetUpdates();

}

bool mgnss::plugins::XbotShared::close() {

        for(auto& controller: _controller_ptrs) controller->close();
        _logger_ptr->flush();
        _logger_ptr->close();
        return true;
}
