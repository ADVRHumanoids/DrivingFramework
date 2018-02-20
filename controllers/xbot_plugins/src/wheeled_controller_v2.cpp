#include <mgnss/xbot_plugins/wheeled_controller_v2.h>

//#include <unistd.h>
REGISTER_XBOT_PLUGIN(WheelsV2, mgnss::xbot_plugins::WheelsV2)

bool mgnss::xbot_plugins::WheelsV2::init_control_plugin(
    XBot::Handle::Ptr handle)
{
  YAML::Node config = mwoibn::robot_class::Robot::getConfig(handle->getPathToConfigFile());
  std::string config_file = config["config_file"].as<std::string>();
  config = mwoibn::robot_class::Robot::getConfig(config_file)["modules"]["wheeled_motion"];

  std::string secondary_file = "";
  if (config["secondary_file"])
    secondary_file = config["secondary_file"].as<std::string>();

  /* Save robot to a private member. */
  _robot_ptr.reset(new mwoibn::robot_class::RobotXBotRT(handle->getRobotInterface(), config_file, config["robot"].as<std::string>(), secondary_file, handle->getSharedMemory()));

//  _robot_ptr.reset(new mwoibn::robot_class::RobotXBotRT(
//      handle->getRobotInterface(), handle->getPathToConfigFile(), "robot", handle->getSharedMemory()));


  _controller_ptr.reset(new mwoibn::WheeledMotionEvent(*_robot_ptr, config_file));

  _srv_rt = handle->getRosHandle()->advertiseService("wheels_command", &mgnss::xbot_plugins::WheelsV2::evenstHandler, this);
  _sub_rt = handle->getRosHandle()->subscribe<custom_messages::CustomCmnd>("wheels_support", 1, &mgnss::xbot_plugins::WheelsV2::supportHandler, this);

  t = std::time(nullptr);
  tm = *std::localtime(&t);

  oss << "RT_wheels_log_" << std::put_time(&tm, "%d-%m-%Y %H-%M-%S") << ".txt";

  file.open(oss.str(),  std::ios::out);

  if(file.is_open())
	std::cout << "opened log file " << oss.str() << std::endl;
  else
	std::cout << "couldn't open log file " << oss.str() << std::endl;
  
//  char cwd[1024];
//  if(getcwd(cwd, sizeof(cwd)) != NULL)
//	  std::cout << "working directory\t" << cwd << std::endl;
//  
  file << "time,"
       << "com_x,"      << "com_y,"
       << "e_com_x,"    << "e_com_y,"
       << "r_com_x,"    << "r_com_y,"
       << "cp_1_x,"     << "cp_1_y,"    << "cp_1_z,"
       << "cp_2_x,"     << "cp_2_y,"    << "cp_2_z,"
       << "cp_3_x,"     << "cp_3_y,"    << "cp_3_z,"
       << "cp_4_x,"     << "cp_4_y,"    << "cp_4_z,"
       << "e_cp_1_x,"   << "e_cp_1_y,"  << "e_cp_1_z,"
       << "e_cp_2_x,"   << "e_cp_2_y,"  << "e_cp_2_z,"
       << "e_cp_3_x,"   << "e_cp_3_y,"  << "e_cp_3_z,"
       << "e_cp_4_x,"   << "e_cp_4_y,"  << "e_cp_4_z,"
       << "r_cp_1_x,"   << "r_cp_1_y,"  << "r_cp_1_z,"
       << "r_cp_2_x,"   << "r_cp_2_y,"  << "r_cp_2_z,"
       << "r_cp_3_x,"   << "r_cp_3_y,"  << "r_cp_3_z,"
       << "r_cp_4_x,"   << "r_cp_4_y,"  << "r_cp_4_z,"
       << "st_1,"       << "st_2,"      << "st_3,"        << "st_4,"
       << "e_st_1,"     << "e_st_2,"    << "e_st_3,"      << "e_st_4,"
       << "r_st_1,"     << "r_st_2,"    << "r_st_3,"      << "r_st_4,"
       << "v_x,"        << "v_y,"       << "v_z,"
       << "w_x,"        << "w_y,"       << "w_z,"
       << "b_sp_1,"     << "b_sp_2,"    << "b_sp_3,"      << "b_sp_4,"
       << "b_icm_1,"    << "b_icm_2,"   << "b_icm_3,"     << "b_icm_4,"
       << "v_sp_1,"     << "v_sp_2,"    << "v_sp_3,"      << "v_sp_4,"
       << "v_icm_1,"    << "v_icm_2,"   << "v_icm_3,"     << "v_icm_4,"
       << "tan_sp_1,"   << "tan_sp_2,"  << "tan_sp_3,"    << "tan_sp_4,"
       << "tan_icm_1,"  << "tan_icm_2," << "tan_icm_3,"  << "tan_icm_4,"
       << "r_1,"        << "r_2,"       << "r_3,"         << "r_4,"
       << "ankle_1,"    << "ankle_2,"   << "ankle_3,"     << "ankle_4,"
       << "base_x,"    << "base_y,"   << "base_z"
       << "\n";

  file.flush();

  fmt.precision = 6;
  fmt.coeffSeparator = ", ";
  fmt.rowSeparator = ", ";
  
  _print.setZero(96);



  return true;
}

void mgnss::xbot_plugins::WheelsV2::on_start(double time)
{
	start = time;
	now = time;
    _valid = _robot_ptr->get();

    if (_valid)
    {
      _robot_ptr->updateKinematics();
      _controller_ptr->init();
      _support = _controller_ptr->getSupportReference();
    }
}

void mgnss::xbot_plugins::WheelsV2::on_stop(double time) {

    _controller_ptr->stop();

}


void mgnss::xbot_plugins::WheelsV2::control_loop(double time,
                                                          double period)
{
    _valid = _robot_ptr->get();

    if (!_valid)
      return;


    _robot_ptr->updateKinematics();

    if (!_initialized)
    {
       if(_valid){
      _controller_ptr->init();
      _support = _controller_ptr->getSupportReference();
       }
       if(!_rate){
           _robot_ptr->setRate(period);
           _controller_ptr->setRate();
           _rate = true;
       }
       if(_rate && _valid){
        _initialized = true;
       }

    }


//    std::cout << "support\t" << _support.transpose() << std::endl;
    _controller_ptr->update(_support);
    _robot_ptr->send();
	
	now = time;
    _print.setZero();
    _print[0] =  time - start;
    _print.segment<2>(1)   = _controller_ptr->getCom();
    _print.segment<2>(3)   = _controller_ptr->errorCom();
    _print.segment<2>(5)   = _controller_ptr->refCom();
    _print.segment<3>(7)   = _controller_ptr->getCp(0);
    _print.segment<3>(10)  = _controller_ptr->getCp(1);
    _print.segment<3>(13)  = _controller_ptr->getCp(2);
    _print.segment<3>(16)  = _controller_ptr->getCp(3);
    _print.segment<3>(19)  = _controller_ptr->errorCp(0);
    _print.segment<3>(22)  = _controller_ptr->errorCp(1);
    _print.segment<3>(25)  = _controller_ptr->errorCp(2);
    _print.segment<3>(28)  = _controller_ptr->errorCp(3);
    _print.segment<12>(31) = _controller_ptr->refCp();
    _print.segment<4>(43)  = _controller_ptr->getSteer();
    _print.segment<4>(47)  = _controller_ptr->errorSteer();
    _print.segment<4>(51)  = _controller_ptr->refSteer();
    _print.segment<3>(55)  = _controller_ptr->getLinVel();
    _print.segment<3>(58)  = _controller_ptr->getAngVel();
    _print.segment<4>(61)  = _controller_ptr->getSteerSP();
    _print.segment<4>(65)  = _controller_ptr->getSteerICM();
    _print.segment<4>(69)  = _controller_ptr->getVelSP();
    _print.segment<4>(73)  = _controller_ptr->getVelICM();
    _print.segment<4>(77)  = _controller_ptr->getDampingSP();
    _print.segment<4>(81)  = _controller_ptr->getDampingICM();
    if(_controller_ptr->isResteer()[0])
      _print[85] = 1;
    if(_controller_ptr->isResteer()[1])
      _print[86] = 1;
    if(_controller_ptr->isResteer()[2])
      _print[87] = 1;
    if(_controller_ptr->isResteer()[3])
      _print[88] = 1;
    _print.segment<4>(89) = _controller_ptr->getAnkleYaw();
    _print.segment<3>(93) = _controller_ptr->getBase();

    file << _print.transpose().format(fmt) << "\n";

}

bool mgnss::xbot_plugins::WheelsV2::close() { 
	
	file.flush();
    file.close();
	
	std::cout << "file closed" << std::endl;
	return true; }
