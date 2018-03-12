#include <mgnss/plugins/ros_base.h>
#include <mwoibn/loaders/robot.h>
#include <config.h>

//REGISTER_XBOT_PLUGIN(Base, mgnss::xbot_plugins::Base)

mgnss::plugins::RosBase::RosBase(int argc, char** argv)
{
  //_init(argc, argv);
}
  void mgnss::plugins::RosBase::_init(int argc, char** argv)
{
  _name = _setName();

  ros::init(argc, argv, _name); // initalize node needed for the service // I can put it in the NRT default plugin (?)
  _n.reset(new ros::NodeHandle());

 // ros::NodeHandle n;
}

  bool mgnss::plugins::RosBase::init()
{

    std::string config_file = std::string(DRIVING_FRAMEWORK_WORKSPACE) + "DrivingFramework/locomotion_framework/configs/mwoibn_v2_5.yaml"; // for now, later take it as a parameter

    YAML::Node config = mwoibn::robot_class::Robot::getConfig(config_file);

    //std::string config_file = config["config_file"].as<std::string>();
    config = mwoibn::robot_class::Robot::getConfig(config_file)["modules"][_name];

    std::string secondary_file = "";
    if (config["secondary_file"])
      secondary_file = config["secondary_file"].as<std::string>();

    _robot_ptr.reset(mwoibn::loaders::Robot::create(config_file,  config["robot"].as<std::string>(), secondary_file).release());

//    mgnss::controllers::JointStates controller(robot, path);

    _resetPrt(config_file);
    _initCallbacks();

    _robot_ptr->get();
    _robot_ptr->updateKinematics();

  return true;
}

void mgnss::plugins::RosBase::start()
{
    //start = time;

    _valid = _robot_ptr->get();
    _rate = true;

    if (_valid)
    {
      _robot_ptr->updateKinematics();
      _controller_ptr->init();
      _initialized = true;

    }

}

void mgnss::plugins::RosBase::stop() {
  _controller_ptr->stop();
}

void mgnss::plugins::RosBase::control_loop()
{

    _valid = _robot_ptr->get();

    if (!_valid)
      return;

    _robot_ptr->updateKinematics();

    if (!_initialized)
    {
//      if(!_rate){
       //_setRate(period); // here I may need a controller method
//      }
       if(_valid)
         _controller_ptr->init();

       if(_rate && _valid)
        _initialized = true;
    }

    _controller_ptr->update();
    _controller_ptr->send();
    _robot_ptr->wait();

}

bool mgnss::plugins::RosBase::close() {
  _controller_ptr->close();
  return true; }

