#define FULL_ROBOT

#include <mgnss/nrt_software/plugins/wheeled_motion_event.h>
#include <config.h>
#include <mwoibn/loaders/robot.h>

#include <mgnss/controllers/wheeled_motion_event.h>
#include <mgnss/controllers/wheeled_references_v3.h>
#include <custom_services/updatePDGains.h>

// LOG
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <chrono>
// !LOG


int main(int argc, char** argv)
{


  mgnss::nrt_software::plugins::WheeledMotionEvent controller(argc, argv);
  controller.init();

  //mgnss::controllers::WheeledMotionEvent& wheeld_controller = controller.get();
  //wheeld_controller.init();
  //mwoibn::robot_class::Robot& robot = controller.robot();

  controller.start(ros::Time::now().toSec());

 /*
  std::ostringstream oss;
  std::ofstream file;

  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);

  oss << "wheels_log_" << std::put_time(&tm, "%d-%m-%Y %H-%M-%S") << ".txt";

  file.open(oss.str(),  std::ios::out);

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

  Eigen::IOFormat fmt(6, 0, ", ", ", ", "", "", "", "");
  file.flush();

  mwoibn::VectorN print(96);
  double start = ros::Time::now().toSec();
  double now = ros::Time::now().toSec();
  */
  //  for(int i = 0; i < 2500; i++)
  while (ros::ok())
  {
//      std::cout << i << std::endl;
    controller.control_loop(ros::Time::now().toSec());
/*
    now = ros::Time::now().toSec();
    print.setZero();
    print[0] =  now - start;
    print.segment<2>(1) =  wheeld_controller.getCom();
    print.segment<2>(3) = wheeld_controller.errorCom();
    print.segment<2>(5) = wheeld_controller.refCom();
    print.segment<3>(7) = wheeld_controller.getCp(0);
    print.segment<3>(10) = wheeld_controller.getCp(1);
    print.segment<3>(13) = wheeld_controller.getCp(2);
    print.segment<3>(16) = wheeld_controller.getCp(3);
    print.segment<3>(19) = wheeld_controller.errorCp(0);
    print.segment<3>(22) = wheeld_controller.errorCp(1);
    print.segment<3>(25) = wheeld_controller.errorCp(2);
    print.segment<3>(28) = wheeld_controller.errorCp(3);
    print.segment<12>(31) = wheeld_controller.refCp();
    print.segment<4>(43) = wheeld_controller.getSteer();
    print.segment<4>(47) = wheeld_controller.errorSteer();
    print.segment<4>(51) = wheeld_controller.refSteer();
    print.segment<3>(55) = wheeld_controller.getLinVel();
    print.segment<3>(58) = wheeld_controller.getAngVel();
    print.segment<4>(61) = wheeld_controller.getSteerSP();
    print.segment<4>(65) = wheeld_controller.getSteerICM();
    print.segment<4>(69) = wheeld_controller.getVelSP();
    print.segment<4>(73) = wheeld_controller.getVelICM();
    print.segment<4>(77) = wheeld_controller.getDampingSP();
    print.segment<4>(81) = wheeld_controller.getDampingICM();
    if(wheeld_controller.isResteer()[0])
      print[85] = 1;
    if(wheeld_controller.isResteer()[1])
      print[86] = 1;
    if(wheeld_controller.isResteer()[2])
      print[87] = 1;
    if(wheeld_controller.isResteer()[3])
      print[88] = 1;
    print.segment<4>(89) = wheeld_controller.getAnkleYaw();
    print.segment<3>(93) = wheeld_controller.getBase();

    file << print.transpose().format(fmt) << "\n";
*/
  }
  controller.stop();
  controller.close();

//  file.flush();
//  file.close();
}
