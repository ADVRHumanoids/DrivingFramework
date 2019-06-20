#include "mgnss/plugins/ros_shared.h"

void mgnss::plugins::RosShared::connect(std::string name)
{
        _name = name;
        ros::init(std::map<std::string, std::string>() ,_name); // initalize node needed for the service // I can put it in the NRT default plugin (?)
        _n.reset(new ros::NodeHandle());

}

void mgnss::plugins::RosShared::connect(int argc, char** argv, std::string name)
{
        _name = name;
        ros::init(argc, argv, _name); // initalize node needed for the service // I can put it in the NRT default plugin (?)
        _n.reset(new ros::NodeHandle());

}

void mgnss::plugins::RosShared::_checkConfig(YAML::Node plugin_config, std::string config_file)
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


void mgnss::plugins::RosShared::_initModules(YAML::Node plugin_config){
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

          mgnss::plugins::RosBase* temp = mgnss::plugins::make(lib);
          temp->init(_robot_ptr, _logger_ptr, _n, _shared, name);
          _controller_ptrs.push_back(std::move(temp->plugin().releaseController()));
          std::cout << "RosShared: loaded " << plugin.as<std::string>() << std::endl;
      }
}

std::string mgnss::plugins::RosShared::_loadConfig(YAML::Node& config, YAML::Node& plugin_config)
{
        std::string config_file;

        if (!_n->getParam("/mwoibn_config", config_file) && !_n->getParam("mwoibn_config", config_file))
            throw std::invalid_argument(std::string("ROS plugin init: couldn't read path to configuration file. Please define 'mwoibn_config'."));

        // Read MWOIBN config file
        config = mwoibn::robot_class::Robot::getConfig(config_file);

        plugin_config = mwoibn::robot_class::Robot::getConfig(config_file);

        return config_file;
}


bool mgnss::plugins::RosShared::init()
{
        YAML::Node config, plugin_config;

        std::string config_file = _loadConfig(config, plugin_config);

        _checkConfig(plugin_config, config_file);

        _logger_ptr.reset(new mwoibn::common::RosLogger(_name));
        _initModules(plugin_config["plugins"]);
        std::cout << "RosShared: loaded " << _robot_ptr.size() << " robot abstractions." << std::endl;
        for(auto& robot: _robot_ptr)
          std::cout << "\t" << robot.first << std::endl;

        _logger_ptr->start();

        return true;
}

void mgnss::plugins::RosShared::start(double time)
{
        // _start = time;
        //
        // for(auto& robot: _robot_ptr)  _valid = robot.second->get() && _valid;
        //
        //
        // _rate = true;
        //
        // if (_valid)
        // {
        //   for(auto& robot: _robot_ptr) robot.second->updateKinematics();
        //   for(auto& controller: _controller_ptrs)  controller->init();
        //
        //   _initialized = true;
        // }

        _start = time;
        _rate = true;

        _init(time);

}

void mgnss::plugins::RosShared::stop() {
        for(auto& controller: _controller_ptrs)  controller->stop();
}

void mgnss::plugins::RosShared::control_loop(double time)
{
  // _valid = true;
  // for(auto& robot: _robot_ptr)
  //   _valid = robot.second->get() && _valid;
  //
  //       if (!_valid)
  //               return;
  //
  //
  // if (!_initialized)
  // {
  //         for(auto& robot: _robot_ptr)
  //             robot.second->updateKinematics();
  //
  //         if(_valid){
  //
  //             for(auto& controller: _controller_ptrs) {controller->init();
  //               controller->send();
  //               for(auto& robot: _robot_ptr)
  //                   robot.second->get();
  //             }
  //         }
  //
  //         if(_rate && _valid)  _initialized = true;
  // }

  if (!_initialized) { _init(time); return; }

  // std::cout << "CONTROLER LOOP" << std::endl;

  for(auto& controller: _controller_ptrs){
    // std::cout << "controller\t" << controller->name() << std::endl;

    if(controller->kinematics.get()){
        controller->model().get();
        // if (controller->model().kinematics_update.get()) std::cout << "updateKinematics" << std::endl;
        controller->model().updateKinematics();
    }
    // for(auto& robot: _robot_ptr)
    //     robot.second->get(); // this will cause unecessary update on external feedbacks (think how to separate both), but it should not be expensive only if not updated?

        controller->update();
        controller->send();
        if(controller->modify.get()) controller->model().kinematics_update.set(true);
        // if (controller->model().kinematics_update.get()) std::cout << "modify" << std::endl;
        _logger_ptr->prefix(controller->name());
        controller->log(*_logger_ptr.get(), time-_start);
  }
  _logger_ptr->write();


  _robot_ptr.begin()->second->wait();

  // std::cout << "reset kinematics" << std::endl;
  for(auto& robot: _robot_ptr)
    robot.second->kinematics_update.set(false);

  for(auto& controller: _controller_ptrs){
    if(controller->kinematics.get()) controller->model().kinematics_update.set(true);

  }

}

bool mgnss::plugins::RosShared::close() {

        for(auto& controller: _controller_ptrs) controller->close();
        _logger_ptr->flush();
        _logger_ptr->close();
        return true;
}


void mgnss::plugins::RosShared::_init(double time){

  for(auto& controller: _controller_ptrs){
    if(controller->kinematics.get()){
          if(!controller->model().get()) return;
          controller->model().updateKinematics();
    }
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

void mgnss::plugins::RosShared::_resetUpdates(){
      _robot_ptr.begin()->second->wait(true);

  // std::cout << "reset kinematics" << std::endl;
  for(auto& robot: _robot_ptr)
    robot.second->kinematics_update.set(false);

  for(auto& controller: _controller_ptrs){
    if(controller->kinematics.get()) controller->model().kinematics_update.set(true);
  }
}
