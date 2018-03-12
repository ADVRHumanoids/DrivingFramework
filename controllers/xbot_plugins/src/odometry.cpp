#include <mgnss/xbot_plugins/odometry.h>

REGISTER_XBOT_PLUGIN(Odometry, mgnss::xbot_plugins::Odometry)

/*
bool mgnss::xbot_plugins::Odometry::init_control_plugin(XBot::Handle::Ptr handle)
{

  _logger->add("base_x", _controller_ptr->getRaw()[0]);
  _logger->add("base_y", _controller_ptr->getRaw()[1]);
  _logger->add("base_z", _controller_ptr->getRaw()[2]);
  _logger->add("base_rx", _controller_ptr->getRaw()[3]);
  _logger->add("base_ry", _controller_ptr->getRaw()[4]);
  _logger->add("base_rz", _controller_ptr->getRaw()[5]);
  _logger->add("cp_1_x", _controller_ptr->getContact(0)[0]);
  _logger->add("cp_1_y", _controller_ptr->getContact(0)[1]);
  _logger->add("cp_1_z", _controller_ptr->getContact(0)[2]);
  _logger->add("cp_2_x", _controller_ptr->getContact(1)[0]);
  _logger->add("cp_2_y", _controller_ptr->getContact(1)[1]);
  _logger->add("cp_2_z", _controller_ptr->getContact(1)[2]);
  _logger->add("cp_3_x", _controller_ptr->getContact(2)[0]);
  _logger->add("cp_3_y", _controller_ptr->getContact(2)[1]);
  _logger->add("cp_3_z", _controller_ptr->getContact(2)[2]);
  _logger->add("cp_4_x", _controller_ptr->getContact(3)[0]);
  _logger->add("cp_4_y", _controller_ptr->getContact(3)[1]);
  _logger->add("cp_4_z", _controller_ptr->getContact(3)[2]);

  _logger->add("time", 0.0);
*/

/*
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
*/
/*
void mgnss::xbot_plugins::Odometry::control_loop(double time, double period)
{

  _logger->add("base_x", _controller_ptr->getRaw()[0]);
  _logger->add("base_y", _controller_ptr->getRaw()[1]);
  _logger->add("base_z", _controller_ptr->getRaw()[2]);
  _logger->add("base_rx", _controller_ptr->getRaw()[3]);
  _logger->add("base_ry", _controller_ptr->getRaw()[4]);
  _logger->add("base_rz", _controller_ptr->getRaw()[5]);
  _logger->add("cp_1_x", _controller_ptr->getContact(0)[0]);
  _logger->add("cp_1_y", _controller_ptr->getContact(0)[1]);
  _logger->add("cp_1_z", _controller_ptr->getContact(0)[2]);
  _logger->add("cp_2_x", _controller_ptr->getContact(1)[0]);
  _logger->add("cp_2_y", _controller_ptr->getContact(1)[1]);
  _logger->add("cp_2_z", _controller_ptr->getContact(1)[2]);
  _logger->add("cp_3_x", _controller_ptr->getContact(2)[0]);
  _logger->add("cp_3_y", _controller_ptr->getContact(2)[1]);
  _logger->add("cp_3_z", _controller_ptr->getContact(2)[2]);
  _logger->add("cp_4_x", _controller_ptr->getContact(3)[0]);
  _logger->add("cp_4_y", _controller_ptr->getContact(3)[1]);
  _logger->add("cp_4_z", _controller_ptr->getContact(3)[2]);

  _logger->add("time", time);

    now = time;
    _print.setZero();
    _print[0] =  time - start;
    _print.segment<6>(1) =  _controller_ptr->getRaw();

    _print.segment<6>(7) =  _controller_ptr->getFiltered();

    file << _print.transpose().format(fmt) << "\n";

}
*/

/*
bool mgnss::xbot_plugins::Odometry::close() {

//    _logger->flush();

//  file.flush();
//  file.close();

//  std::cout << "odometry log closed" << std::endl;
  return true; }
*/
