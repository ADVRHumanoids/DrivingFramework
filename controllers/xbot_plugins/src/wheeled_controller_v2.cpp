#include <mgnss/xbot_plugins/wheeled_controller_v2.h>

REGISTER_XBOT_PLUGIN(WheelsV2, mgnss::xbot_plugins::WheelsV2)
/*
bool mgnss::xbot_plugins::WheelsV2::init_control_plugin(
    XBot::Handle::Ptr handle)
{
  _logger = XBot::MatLogger::getLogger("/tmp/WheelsV2_logger");

  _logger->add("e_base_z", _controller_ptr->getBaseError()[2]);
  _logger->add("r_base_z", _controller_ptr->getBodyPosition()[2]);

  _logger->add("e_base_rx", _controller_ptr->getBaseOrnError()[0]);
  _logger->add("e_base_ry", _controller_ptr->getBaseOrnError()[1]);
  _logger->add("e_base_rz", _controller_ptr->getBaseOrnError()[2]);
  _logger->add("base_rx", _robot_ptr->state.get()[3]);
  _logger->add("base_ry", _robot_ptr->state.get()[4]);
  _logger->add("base_rz", _robot_ptr->state.get()[5]);

  _logger->add("base_x", _robot_ptr->state.get()[0]);
  _logger->add("base_y", _robot_ptr->state.get()[1]);
  _logger->add("base_z", _robot_ptr->state.get()[2]);

  _logger->add("com_x", _controller_ptr->getCom()[0]);
  _logger->add("com_y", _controller_ptr->getCom()[1]);
  _logger->add("r_com_x", _controller_ptr->refCom()[0]);
  _logger->add("r_com_y", _controller_ptr->refCom()[1]);

//  _logger->add("r_1", _controller_ptr->isResteer()[0]);
//  _logger->add("r_2", _controller_ptr->isResteer()[1]);
//  _logger->add("r_3", _controller_ptr->isResteer()[2]);
//  _logger->add("r_4", _controller_ptr->isResteer()[3]);

  _logger->add("cp_1_x", _controller_ptr->getCp(0)[0]);
  _logger->add("cp_1_y", _controller_ptr->getCp(0)[1]);
//  _logger->add("cp_1_z", _controller_ptr->getCp(0)[2]);
  _logger->add("cp_2_x", _controller_ptr->getCp(1)[0]);
  _logger->add("cp_2_y", _controller_ptr->getCp(1)[1]);
//  _logger->add("cp_2_z", _controller_ptr->getCp(1)[2]);
  _logger->add("cp_3_x", _controller_ptr->getCp(2)[0]);
  _logger->add("cp_3_y", _controller_ptr->getCp(2)[1]);
//  _logger->add("cp_3_z", _controller_ptr->getCp(2)[2]);
  _logger->add("cp_4_x", _controller_ptr->getCp(3)[0]);
  _logger->add("cp_4_y", _controller_ptr->getCp(3)[1]);
//  _logger->add("cp_4_z", _controller_ptr->getCp(3)[2]);

  _logger->add("r_cp_1_x", _controller_ptr->refCp()[0]);
  _logger->add("r_cp_1_y", _controller_ptr->refCp()[1]);
//  _logger->add("r_cp_1_z", _controller_ptr->refCp()[2]);
  _logger->add("r_cp_2_x", _controller_ptr->refCp()[3]);
  _logger->add("r_cp_2_y", _controller_ptr->refCp()[4]);
//  _logger->add("r_cp_2_z", _controller_ptr->refCp()[5]);
  _logger->add("r_cp_3_x", _controller_ptr->refCp()[6]);
  _logger->add("r_cp_3_y", _controller_ptr->refCp()[7]);
//  _logger->add("r_cp_3_z", _controller_ptr->refCp()[8]);
  _logger->add("r_cp_4_x", _controller_ptr->refCp()[9]);
  _logger->add("r_cp_4_y", _controller_ptr->refCp()[10]);
//  _logger->add("r_cp_4_z", _controller_ptr->refCp()[11]);

  _logger->add("r_st_1", _controller_ptr->refSteer()[0]);
  _logger->add("r_st_2", _controller_ptr->refSteer()[1]);
  _logger->add("r_st_3", _controller_ptr->refSteer()[2]);
  _logger->add("r_st_4", _controller_ptr->refSteer()[3]);
  _logger->add("st_1", _controller_ptr->getSteer()[0]);
  _logger->add("st_2", _controller_ptr->getSteer()[1]);
  _logger->add("st_3", _controller_ptr->getSteer()[2]);
  _logger->add("st_4", _controller_ptr->getSteer()[3]);

  _logger->add("tan_sp_1", _controller_ptr->getDampingSP()[0]);
  _logger->add("tan_sp_2", _controller_ptr->getDampingSP()[1]);
  _logger->add("tan_sp_3", _controller_ptr->getDampingSP()[2]);
  _logger->add("tan_sp_4", _controller_ptr->getDampingSP()[3]);

  _logger->add("tan_icm_1", _controller_ptr->getDampingICM()[0]);
  _logger->add("tan_icm_2", _controller_ptr->getDampingICM()[1]);
  _logger->add("tan_icm_3", _controller_ptr->getDampingICM()[2]);
  _logger->add("tan_icm_4", _controller_ptr->getDampingICM()[3]);

//  _logger->add("ankle_yaw_1", _robot_ptr->state.get()[10]);
//  _logger->add("ankle_yaw_2", _robot_ptr->state.get()[16]);
//  _logger->add("ankle_yaw_3", _robot_ptr->state.get()[22]);
//  _logger->add("ankle_yaw_4", _robot_ptr->state.get()[28]);
//  _logger->add("e_st_1", _controller_ptr->errorSteer()[0]);
//  _logger->add("e_st_2", _controller_ptr->errorSteer()[1]);
//  _logger->add("e_st_3", _controller_ptr->errorSteer()[2]);
//  _logger->add("e_st_4", _controller_ptr->errorSteer()[3]);

  _logger->add("time", 0.0);


//  t = std::time(nullptr);
//  tm = *std::localtime(&t);

//  oss << "RT_wheels_log_" << std::put_time(&tm, "%d-%m-%Y %H-%M-%S") << ".txt";

//  file.open(oss.str(),  std::ios::out);

//  if(file.is_open())
//	std::cout << "opened log file " << oss.str() << std::endl;
//  else
//	std::cout << "couldn't open log file " << oss.str() << std::endl;

//  char cwd[1024];
//  if(getcwd(cwd, sizeof(cwd)) != NULL)
//    std::cout << "working directory\t" << cwd << std::endl;
//
*/
/*  file << "time,"
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
*/


//  return true;
//}

bool mgnss::xbot_plugins::WheelsV2::init_control_plugin(XBot::Handle::Ptr handle){
  mgnss::plugins::XbotBase::init_control_plugin(handle);
  _support = get().getSupportReference();
  return true;
}

void mgnss::xbot_plugins::WheelsV2::on_start(double time)
{
//	now = time;
    mgnss::plugins::XbotBase::on_start(time);
    if (_valid)
      _support.noalias() = get().getSupportReference();

}

void mgnss::xbot_plugins::WheelsV2::control_loop(double time, double period)
{

    _valid = _robot_ptr->get();

    if (!_valid)
      return;

    _robot_ptr->updateKinematics();

    if (!_initialized)
    {
      if(!_rate){
       _setRate(period);
       _rate = true;
      }
       _valid = _robot_ptr->feedbacks.reset();
       if(_valid){

         _controller_ptr->init();
       }
       if(_rate && _valid)
        _initialized = true;
    }

    _controller_ptr->update();
    _controller_ptr->send();

//   std::cout <<  _robot_ptr->command.get(mwoibn::robot_class::INTERFACE::VELOCITY).transpose() << std::endl;

}


/*
void mgnss::xbot_plugins::WheelsV2::control_loop(double time,
                                                          double period)
{
*/

/*
  _logger->add("e_base_z", _controller_ptr->getBaseError()[2]);
  _logger->add("r_base_z", _controller_ptr->getBodyPosition()[2]);

  _logger->add("e_base_rx", _controller_ptr->getBaseOrnError()[0]);
  _logger->add("e_base_ry", _controller_ptr->getBaseOrnError()[1]);
  _logger->add("e_base_rz", _controller_ptr->getBaseOrnError()[2]);
  _logger->add("base_rx", _robot_ptr->state.get()[3]);
  _logger->add("base_ry", _robot_ptr->state.get()[4]);
  _logger->add("base_rz", _robot_ptr->state.get()[5]);

  _logger->add("base_x", _robot_ptr->state.get()[0]);
  _logger->add("base_y", _robot_ptr->state.get()[1]);
  _logger->add("base_z", _robot_ptr->state.get()[2]);

  _logger->add("com_x", _controller_ptr->getComFull()[0]);
  _logger->add("com_y", _controller_ptr->getComFull()[1]);

  _logger->add("r_com_x", _controller_ptr->refCom()[0]);
  _logger->add("r_com_y", _controller_ptr->refCom()[1]);

//  _logger->add("r_1", _controller_ptr->isResteer()[0]);
//  _logger->add("r_2", _controller_ptr->isResteer()[1]);
//  _logger->add("r_3", _controller_ptr->isResteer()[2]);
//  _logger->add("r_4", _controller_ptr->isResteer()[3]);

  _log_point = _controller_ptr->getCp(0);
  _logger->add("cp_1_x", _log_point[0]);
  _logger->add("cp_1_y", _log_point[1]);
//  _logger->add("cp_1_z", _log_point[2]);
  _log_point = _controller_ptr->getCp(1);
  _logger->add("cp_2_x", _log_point[0]);
  _logger->add("cp_2_y", _log_point[1]);
//  _logger->add("cp_2_z", _log_point[2]);
  _log_point = _controller_ptr->getCp(2);
  _logger->add("cp_3_x", _log_point[0]);
  _logger->add("cp_3_y", _log_point[1]);
//  _logger->add("cp_3_z", _log_point[2]);
  _log_point = _controller_ptr->getCp(3);
  _logger->add("cp_4_x", _log_point[0]);
  _logger->add("cp_4_y", _log_point[1]);
//  _logger->add("cp_4_z", _log_point[2]);
  _logger->add("r_cp_1_x", _controller_ptr->refCp()[0]);
  _logger->add("r_cp_1_y", _controller_ptr->refCp()[1]);
//  _logger->add("r_cp_1_z", _controller_ptr->refCp()[2]);
  _logger->add("r_cp_2_x", _controller_ptr->refCp()[3]);
  _logger->add("r_cp_2_y", _controller_ptr->refCp()[4]);
//  _logger->add("r_cp_2_z", _controller_ptr->refCp()[5]);
  _logger->add("r_cp_3_x", _controller_ptr->refCp()[6]);
  _logger->add("r_cp_3_y", _controller_ptr->refCp()[7]);
//  _logger->add("r_cp_3_z", _controller_ptr->refCp()[8]);
  _logger->add("r_cp_4_x", _controller_ptr->refCp()[9]);
  _logger->add("r_cp_4_y", _controller_ptr->refCp()[10]);
//  _logger->add("r_cp_4_z", _controller_ptr->refCp()[11]);
  _logger->add("r_st_1", _controller_ptr->refSteer()[0]);
  _logger->add("r_st_2", _controller_ptr->refSteer()[1]);
  _logger->add("r_st_3", _controller_ptr->refSteer()[2]);
  _logger->add("r_st_4", _controller_ptr->refSteer()[3]);
  _logger->add("st_1", _controller_ptr->getSteer()[0]);
  _logger->add("st_2", _controller_ptr->getSteer()[1]);
  _logger->add("st_3", _controller_ptr->getSteer()[2]);
  _logger->add("st_4", _controller_ptr->getSteer()[3]);
//  _logger->add("e_st_1", _controller_ptr->errorSteer()[0]);
//  _logger->add("e_st_2", _controller_ptr->errorSteer()[1]);
//  _logger->add("e_st_3", _controller_ptr->errorSteer()[2]);
//  _logger->add("e_st_4", _controller_ptr->errorSteer()[3]);

  _logger->add("tan_sp_1", _controller_ptr->getDampingSP()[0]);
  _logger->add("tan_sp_2", _controller_ptr->getDampingSP()[1]);
  _logger->add("tan_sp_3", _controller_ptr->getDampingSP()[2]);
  _logger->add("tan_sp_4", _controller_ptr->getDampingSP()[3]);

  _logger->add("tan_icm_1", _controller_ptr->getDampingICM()[0]);
  _logger->add("tan_icm_2", _controller_ptr->getDampingICM()[1]);
  _logger->add("tan_icm_3", _controller_ptr->getDampingICM()[2]);
  _logger->add("tan_icm_4", _controller_ptr->getDampingICM()[3]);

  //_logger->add("ankle_yaw_1", _robot_ptr->state.get()[10]);
  //_logger->add("ankle_yaw_2", _robot_ptr->state.get()[16]);
  //_logger->add("ankle_yaw_3", _robot_ptr->state.get()[22]);
  //_logger->add("ankle_yaw_4", _robot_ptr->state.get()[28]);

  _logger->add("time", time-start);

  /*
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
    _print.segment<3>(93) = _robot.state.get().head<3>();

    file << _print.transpose().format(fmt) << "\n";
*/
//}
/*
bool mgnss::xbot_plugins::WheelsV2::close() {

//    _logger->flush();

//    file.flush();
//    file.close();

//	std::cout << "file closed" << std::endl;
	return true; }
*/
