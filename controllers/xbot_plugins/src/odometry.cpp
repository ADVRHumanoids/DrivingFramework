#include <mgnss/xbot_plugins/odometry.h>

REGISTER_XBOT_PLUGIN(Odometry, mgnss::xbot_plugins::Odometry)

bool mgnss::xbot_plugins::Odometry::init_control_plugin(XBot::Handle::Ptr handle)
{
  /* Save robot to a private member. */
  YAML::Node config = mwoibn::robot_class::Robot::getConfig(handle->getPathToConfigFile());
  std::string config_file = config["config_file"].as<std::string>();
  config = mwoibn::robot_class::Robot::getConfig(config_file)["modules"]["odometry"];

  std::string secondary_file = "";
  if (config["secondary_file"])
    secondary_file = config["secondary_file"].as<std::string>();

  /* Save robot to a private member. */
  _robot_ptr.reset(new mwoibn::robot_class::RobotXBotRT(handle->getRobotInterface(), config_file, config["robot"].as<std::string>(), secondary_file, handle->getSharedMemory()));

//  _robot_ptr.reset(new mwoibn::robot_class::RobotXBotRT(handle->getRobotInterface(), handle->getPathToConfigFile(), "robot2", handle->getSharedMemory()));

  _controller_ptr.reset(new mgnss::odometry::Odometry(*_robot_ptr, {"wheel_1", "wheel_2", "wheel_3", "wheel_4"}, 0.078));

  _robot_ptr->update();

  t = std::time(nullptr);
  tm = *std::localtime(&t);

  oss << "RT_odometry_log_" << std::put_time(&tm, "%d-%m-%Y %H-%M-%S") << ".txt";

  file.open(oss.str(),  std::ios::out);

  if(file.is_open())
        std::cout << "opened log file " << oss.str() << std::endl;
  else
        std::cout << "couldn't open log file " << oss.str() << std::endl;

  char cwd[1024];
  if(getcwd(cwd, sizeof(cwd)) != NULL)
    std::cout << "working directory\t" << cwd << std::endl;
  file << "time,"
       << "raw_x,"      << "raw_y,"     << "raw_z,"
       << "raw_tx,"     << "raw_ty,"    << "raw_tz,"
       << "fil_x,"      << "fil_y,"     << "fil_z,"
       << "fil_tx,"     << "fil_ty,"    << "fil_tz,"
       << "\n";

  file.flush();

  fmt.precision = 6;
  fmt.coeffSeparator = ", ";
  fmt.rowSeparator = ", ";

  _print.setZero(13);

  return true;
}

void mgnss::xbot_plugins::Odometry::on_start(double time)
{
  start = time;
  now = time;
    _valid = _robot_ptr->get();

    if (_valid)
    {
      _robot_ptr->updateKinematics();
//      _controller_ptr->init();
    }
}

void mgnss::xbot_plugins::Odometry::on_stop(double time) {}

void mgnss::xbot_plugins::Odometry::control_loop(double time, double period)
{
    _valid = _robot_ptr->get();

    if (!_valid)
      return;

    _robot_ptr->updateKinematics();

    if (!_initialized)
    {
      if(!_rate){
          _robot_ptr->setRate(period);
          _controller_ptr->setRate();
       _rate = true;
      }
       if(_valid)
      _controller_ptr->init();

       if(_rate && _valid)
        _initialized = true;
    }

    _controller_ptr->update();
    _robot_ptr->send();

    now = time;
    _print.setZero();
    _print[0] =  time - start;
    _print.segment<6>(1) =  _controller_ptr->getRaw();

    _print.segment<6>(7) =  _controller_ptr->getFiltered();

    file << _print.transpose().format(fmt) << "\n";
}

bool mgnss::xbot_plugins::Odometry::close() {

  file.flush();
  file.close();

  std::cout << "odometry log closed" << std::endl;
  return true; }

