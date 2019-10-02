#include "mgnss/plugins/xbot_shared.h"

// Register the plugin
REGISTER_XBOT_PLUGIN_( mgnss::plugins::XbotShared ) \

void mgnss::plugins::XbotShared::connect(XBot::Handle::Ptr handle)
{
        _name = "shared";
        _n = handle->getRosHandle();
}

void mgnss::plugins::XbotShared::_checkConfig(YAML::Node plugin_config, std::string config_file)
{
        // check if all the expected tags have been defined in the config file
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

      // load the requested plugins and the configuration name. They are given in a format registered_plugin::configuration_name
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

          // generate the plugin
          mgnss::plugins::XbotBaseUnify* temp = mgnss::plugins::xbot_make(lib);
          // initialize the plugin
          temp->init_control_plugin(handle, _robot_ptr, _logger_ptr, _n, _shared, name);
          // move the module to the shared space
         _controller_ptrs.push_back(std::move(temp->plugin().releaseController()));
          std::cout << "XbotShared: loaded " << plugin.as<std::string>() << std::endl;
      }
}

std::string mgnss::plugins::XbotShared::_loadConfig(XBot::Handle::Ptr handle, YAML::Node& config, YAML::Node& plugin_config)
{
        // read the XBotCore configuration file
        YAML::Node xbot_config = mwoibn::robot_class::Robot::getConfig(handle->getPathToConfigFile());

        // Read path to config file from the XBotCore configuration file
        if (!xbot_config["config_file"])
                throw std::invalid_argument(handle->getPathToConfigFile() +
                                            std::string("\t Please specify MWOIBN config file."));

        std::string config_file = xbot_config["config_file"].as<std::string>();

        // Read the configuration file
        config = mwoibn::robot_class::Robot::getConfig(config_file);

        // Read the plugin configuration
        plugin_config = mwoibn::robot_class::Robot::getConfig(config_file);

        return config_file;
}


bool mgnss::plugins::XbotShared::init_control_plugin(XBot::Handle::Ptr handle)
{
        YAML::Node config, plugin_config;

        connect(handle);
        std::string config_file = _loadConfig(handle, config, plugin_config);

        _checkConfig(plugin_config, config_file);

        // generate the logger
        _logger_ptr.reset(new mwoibn::common::XbotLogger(_name));
        // init all the plugins/controller
        _initModules(handle, plugin_config["plugins"]);
        // report the loaded robot instances
        std::cout << "XbotShared: loaded " << _robot_ptr.size() << " robot abstractions." << std::endl;
        for(auto& robot: _robot_ptr)
          std::cout << "\t" << robot.first << std::endl;


        // allocate the logger memmory
        _logger_ptr->start();

        return true;
}

void mgnss::plugins::XbotShared::on_start(double time)
{
        _start = time; // log the initialization time
        _rate = true;

        _init(time);
}

void mgnss::plugins::XbotShared::_init(double time){

  for(auto& controller: _controller_ptrs){
    if(!controller->model().get()) return; // update the feedbacks, check if no feedback reported error, stop the initialization if an error was detected
    if(controller->kinematics.get()) controller->model().updateKinematics(); // update kinematics if necessary

    controller->init(); // init the plugin
    controller->send(); //set module outputs (e.g. desired state, estimate forces)
    if(controller->modify.get()) controller->model().kinematics_update.set(true);  // set update flags for the robot

    _logger_ptr->prefix(controller->name()); // set logger prefix to the curren module
    controller->log(*_logger_ptr.get(), time-_start);  // log the data
        }

  _logger_ptr->write();  // write the data to file in NRT, in RT XBotCore this function does not do anything.
  _resetUpdates(); // reset the flags releated to kineamtic updates
  if(_rate)  _initialized = true; // change the plugin status to initialized

}

 // reset the kineamtic updates flags
void mgnss::plugins::XbotShared::_resetUpdates(){
      _robot_ptr.begin()->second->wait(true);

  for(auto& robot: _robot_ptr)
    robot.second->kinematics_update.set(false);

  for(auto& controller: _controller_ptrs){
    if(controller->kinematics.get()) controller->model().kinematics_update.set(true);
  }
}

void mgnss::plugins::XbotShared::on_stop(double time) {
        // stop all the modules
        for(auto& controller: _controller_ptrs)  controller->stop();
}

void mgnss::plugins::XbotShared::control_loop(double time)
{
    

  // make sure that all the modules initialized correctly. If not, rerun initialization.
  if (!_initialized) { _init(time); return; }
  
 
  for(auto& controller: _controller_ptrs){

    if(!controller->model().get()) return; // update the feedbacks, check if no feedback reported error, stop the initialization if an error was detected

    if(controller->kinematics.get())
        controller->model().updateKinematics(); // update kinematics if required


    controller->update();  // update the module
    controller->send(); // //set outputs from the module (e.g. desired state, estimate forces)
    if(controller->modify.get()) controller->model().kinematics_update.set(true);  // set update flags for the robot
    _logger_ptr->prefix(controller->name()); // set logger prefix to the curren plugin
    controller->log(*_logger_ptr.get(), time-_start); // log the data

  }
  
  _logger_ptr->write();  // write the data to file in NRT, in RT XBotCore this function does not do anything.

  _resetUpdates();  // reset the flags releated to kineamtic updates

}

bool mgnss::plugins::XbotShared::close() {

        for(auto& controller: _controller_ptrs) controller->close(); // shut down the modules
        _logger_ptr->flush(); // save the data to the file, release the logger memmory
        _logger_ptr->close(); // close the log file
        return true;
}
