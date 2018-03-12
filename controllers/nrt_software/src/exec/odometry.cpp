#include <mgnss/nrt_software/plugins/odometry.h>

//// LOG
//#include <iostream>
//#include <fstream>
//#include <iomanip>
//#include <ctime>
//#include <chrono>
//// !LOG

int main(int argc, char** argv)
{

  mgnss::nrt_software::plugins::Odometry controller(argc, argv);

  controller.init();
  controller.start();

/*
  std::ostringstream oss;
  std::ofstream file;

  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);

  oss << "odometry_log_" << std::put_time(&tm, "%d-%m-%Y %H-%M-%S") << ".txt";

  file.open(oss.str(),  std::ios::out);
  file << "time,"
       << "raw_x,"      << "raw_y,"     << "raw_z,"
       << "raw_tx,"     << "raw_ty,"    << "raw_tz,"
       << "fil_x,"      << "fil_y,"     << "fil_z,"
       << "fil_tx,"     << "fil_ty,"    << "fil_tz,"
       << "\n";

  Eigen::IOFormat fmt(6, 0, ", ", ", ", "", "", "", "");
  file.flush();

  mwoibn::VectorN print(13);
  double start = ros::Time::now().toSec();
  double now = ros::Time::now().toSec();
*/


  //  for(int i = 0; i < 2500; i++)
  while (ros::ok())
  {
    controller.control_loop();

/*    now = ros::Time::now().toSec();
    print.setZero();
    print[0] =  now - start;
    print.segment<6>(1) =  odometry.getRaw();
    print.segment<6>(7) =  odometry.getFiltered();

    file << print.transpose().format(fmt) << "\n";
    */
  }

  controller.close();
//  file.flush();
//  file.close();

}
