#include "mgnss/plugins/ros_shared.h"

void mgnss::plugins::RosShared::connect(std::string name)
{
        _name = name;
        ros::init(std::map<std::string, std::string>() ,_name); // initalize node
        _n.reset(new ros::NodeHandle());

}

void mgnss::plugins::RosShared::connect(int argc, char** argv, std::string name)
{
        _name = name;
        ros::init(argc, argv, _name); // initalize node
        _n.reset(new ros::NodeHandle());

}

void mgnss::plugins::RosShared::_checkConfig(YAML::Node plugin_config, std::string config_file)
{
        // check if confiuration has all the expected tags
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


void mgnss::plugins::RosShared::_initModules(YAML::Node plugin_config){
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
          mgnss::plugins::RosBase* temp = mgnss::plugins::make(lib);
          // initialize the plugin
          temp->init(_robot_ptr, _logger_ptr, _n, _shared, name);
          // move the module to the shared space
          _controller_ptrs.push_back(std::move(temp->plugin().releaseController()));
          std::cout << "RosShared: loaded " << plugin.as<std::string>() << std::endl;
      }
}

std::string mgnss::plugins::RosShared::_loadConfig(YAML::Node& config, YAML::Node& plugin_config)
{
        std::string config_file;

        // Read the oath to the configuration file from the ROS parameter server
        if (!_n->getParam("/mwoibn_config", config_file) && !_n->getParam("mwoibn_config", config_file))
            throw std::invalid_argument(std::string("ROS plugin init: couldn't read path to configuration file. Please define 'mwoibn_config'."));

        // Read the configuration
        config = mwoibn::robot_class::Robot::getConfig(config_file);

        // Read the plugin specific configuration
        plugin_config = mwoibn::robot_class::Robot::getConfig(config_file);

        return config_file;
}


bool mgnss::plugins::RosShared::init()
{
        YAML::Node config, plugin_config;

        std::string config_file = _loadConfig(config, plugin_config);

        _checkConfig(plugin_config, config_file);

        // create the logger
        _logger_ptr.reset(new mwoibn::common::RosLogger(_name));
        // init all the modules
        _initModules(plugin_config["plugins"]);
        // report the loaded robot instances
        std::cout << "RosShared: loaded " << _robot_ptr.size() << " robot abstractions." << std::endl;
        for(auto& robot: _robot_ptr)
          std::cout << "\t" << robot.first << std::endl;

        // allocate the memmory for the logger
        _logger_ptr->start();

        return true;
}

void mgnss::plugins::RosShared::start(double time)
{


        _start = time; // log the initialization time
        _rate = true;

        _init(time);

}

void mgnss::plugins::RosShared::stop() {
        // stop all the modules
        for(auto& controller: _controller_ptrs)  controller->stop();
}

void mgnss::plugins::RosShared::control_loop(double time)
{



  // check if all the modules initialized successfully. If not, run initialization again.
  if (!_initialized) { _init(time); return; }


  for(auto& controller: _controller_ptrs){

    controller->model().get(); // update the feedbacks

    if(controller->kinematics.get()) controller->model().updateKinematics(); // update kinematics if required

        controller->update(); // update the module
        controller->send(); // set all the module outputs (e.g. desired state, estimate forces)

        if(controller->modify.get()) controller->model().kinematics_update.set(true); // set update flags for the robot

        _logger_ptr->prefix(controller->name()); // set logger prefix to the current module
        controller->log(*_logger_ptr.get(), time-_start); // log the data
  }
  _logger_ptr->write(); // save the data to file


 // reset the flags releated to kineamtic updates
  _resetUpdates();

}

bool mgnss::plugins::RosShared::close() {

        // shut down all the modules
        for(auto& controller: _controller_ptrs) controller->close();
        _logger_ptr->flush(); // release the logger memmory
        _logger_ptr->close(); // close the log file
        return true;
}


void mgnss::plugins::RosShared::_init(double time){

  for(auto& controller: _controller_ptrs){
    if(controller->kinematics.get()){
          if(!controller->model().get()) return;  // update the feedbacks, check if no feedback reported error, stop the initialization if an error was detected
          controller->model().updateKinematics(); // update kinematics if necessary
    }
    controller->init(); // init the plugin
    controller->send(); // set all the module outputs (e.g. desired state, estimate forces)

    if(controller->modify.get()) controller->model().kinematics_update.set(true); // set update flags for the robot
    _logger_ptr->prefix(controller->name()); // set logger prefix to the curren module
    controller->log(*_logger_ptr.get(), time-_start);  // log the data
  }

  _logger_ptr->write(); // save the data to file
  _resetUpdates(); // reset the flags releated to kineamtic updates
  if(_rate)  _initialized = true; // change the plugin status to initialized

}

 // reset the kinematic updates flags
void mgnss::plugins::RosShared::_resetUpdates(){
      _robot_ptr.begin()->second->wait(true);

  for(auto& robot: _robot_ptr)
    robot.second->kinematics_update.set(false);

  for(auto& controller: _controller_ptrs){
    if(controller->kinematics.get()) controller->model().kinematics_update.set(true);
  }
}
