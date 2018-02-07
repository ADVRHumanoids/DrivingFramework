#define FULL_ROBOT
//#define TWO_ROBOTS

#include <config.h>
#include <mwoibn/loaders/robot.h>

#ifdef FULL_ROBOT
#include <mgnss/controllers/wheeled_motion_event.h>
#endif
#ifdef TWO_ROBOTS
#include <mgnss/controllers/wheeled_motion_event_v2.h>
#endif
#include <mgnss/controllers/wheeled_references_v3.h>
#include <custom_services/updatePDGains.h>

#ifdef TWO_ROBOTS
bool evenstHandler(custom_services::updatePDGains::Request& req,
                   custom_services::updatePDGains::Response& res,
                   mwoibn::SupportPolygon3* support,
                   mwoibn::WheeledMotionEvent2* controller);
#endif
#ifdef FULL_ROBOT
bool evenstHandler(custom_services::updatePDGains::Request& req,
                   custom_services::updatePDGains::Response& res,
                   mwoibn::SupportPolygon3* support,
                   mwoibn::WheeledMotionEvent* controller);
#endif

// LOG
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <chrono>
// !LOG


int main(int argc, char** argv)
{
  ros::init(argc, argv, "wheels_reference"); // initalize node

  ros::NodeHandle n;

// init wheels_controller

#ifdef FULL_ROBOT
  mwoibn::loaders::Robot loader;
#endif
#ifdef TWO_ROBOTS
  mwoibn::loaders::Robot loader, loader_2;
#endif

  mwoibn::robot_class::Robot& robot = loader.init(
      std::string(DRIVING_FRAMEWORK_WORKSPACE) +
          "DrivingFramework/"
          "locomotion_framework/configs/mwoibn_v2.yaml",
      "default", std::string(DRIVING_FRAMEWORK_WORKSPACE) +
                     "DrivingFramework/"
                     "locomotion_framework/configs/lower_body.yaml");
#ifdef FULL_ROBOT
  mwoibn::WheeledMotionEvent wheeld_controller(robot);
  wheeld_controller.init();
#endif
#ifdef TWO_ROBOTS
  mwoibn::robot_class::Robot& full_robot =
      loader_2.init(std::string(DRIVING_FRAMEWORK_WORKSPACE) +
                        "DrivingFramework/"
                        "locomotion_framework/configs/mwoibn_v2.yaml",
                    "full_model");

  //  mwoibn::robot_class::Robot& full_robot = robot;
  robot.addBiMap(full_robot, "full_body");

  mwoibn::WheeledMotionEvent2 wheeld_controller(robot, full_robot);
#endif

  mwoibn::SupportPolygon3 support(0.45, 0.22, 0.078, robot.rate());

  //  support.setBase(0.25, 0.125); // HARDCODED
  mwoibn::VectorN base(12);

  base << 0.15, 0.125, 0.0, 0.15, -0.125, 0.0, -0.25, 0.125, 0.0, -0.25, -0.125,
      0.0;
  support.setBase(base);
  support.setUpperLimit(-15 * 3.1416 / 180);

  support.setLowerLimit(-65 * 3.1416 / 180);
  support.setRadious(0.38);
  support.setStep(0.002);

  //        mwoibn::Base base;

  //  base.heading.setUpperLimit(2 * 3.1416 / 180);
  //  base.heading.setLowerLimit(-2 * 3.1416 / 180);
  // ros topics/service support
  ros::ServiceServer service =
      n.advertiseService<custom_services::updatePDGains::Request,
                         custom_services::updatePDGains::Response>(
          "wheels_command",
          boost::bind(&evenstHandler, _1, _2, &support, &wheeld_controller));

  // starting
  support.setCurrent(wheeld_controller.getSupportReference());
  support.setDesired(wheeld_controller.getSupportReference());

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
  //  for(int i = 0; i < 2500; i++)
  while (ros::ok())
  {
    //      std::cout << i << std::endl;
    support.update();
    wheeld_controller.fullUpdate(support.get());
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

  }

#ifdef FULL_ROBOT
  wheeld_controller.stop();
#endif
  file.flush();
  file.close();
}

#ifdef FULL_ROBOT
bool evenstHandler(custom_services::updatePDGains::Request& req,
                   custom_services::updatePDGains::Response& res,
                   mwoibn::SupportPolygon3* support,
                   mwoibn::WheeledMotionEvent* controller)
#endif
#ifdef TWO_ROBOTS
    bool evenstHandler(custom_services::updatePDGains::Request& req,
                       custom_services::updatePDGains::Response& res,
                       mwoibn::SupportPolygon3* support,
                       mwoibn::WheeledMotionEvent2* controller)
#endif
{
  //  std::cout << "req\t" << req.p << "\t" << req.d << "\t" << req.nr <<
  //  std::endl;

  if (req.p == 1)
  { // base
    if (req.d == 1)
      controller->setBaseDotX(req.nr / 100.0);
    else if (req.d == 2)
      controller->setBaseDotY(req.nr / 100.0);
    else if (req.d == 3)
      controller->setBaseDotZ(req.nr / 100.0);
    else if (req.d == 4)
    {
      controller->setBaseDotHeading(req.nr / 100.0);
    }
    else if (req.d == 5)
      controller->setBaseRotVelX(req.nr / 100.0);
    else if (req.d == 6)
      controller->setBaseRotVelY(req.nr / 100.0);
    else
    {
      res.success = false;
      return false;
    }
    res.success = true;
    return true;
  }
  if (req.p == 2)
  { // base
    if (req.d == 1)
      controller->setBaseX(req.nr / 100.0);
    else if (req.d == 2)
      controller->setBaseY(req.nr / 100.0);
    else if (req.d == 3)
      controller->setBaseZ(req.nr / 100.0);
    else if (req.d == 4)
      controller->setBaseHeading(req.nr / 100.0);
    else if (req.d == 5)
      controller->rotateBaseX(req.nr / 100.0);
    else if (req.d == 6)
      controller->rotateBaseY(req.nr / 100.0);
    else if (req.d == 7)
      controller->resteer(req.nr);
    else
    {
      res.success = false;
      return false;
    }
    res.success = true;
    return true;
  }
  if (req.p == 3)
  { // support
    mwoibn::SUPPORT_MOTION motion;
    mwoibn::SUPPORT_STATE state;

    if (req.d >= 0 && req.d < 3)
    {
      motion = static_cast<mwoibn::SUPPORT_MOTION>(req.d);

      if (req.nr > 0 && req.nr < 4)
      {
        state = static_cast<mwoibn::SUPPORT_STATE>(req.nr);
        support->initMotion(motion, state);
      }
      else{
        support->changeMotion(motion);
      }

      support->setModeX(0, mwoibn::SUPPORT_INTERFACE::POSITION);
      support->setModeX(1, mwoibn::SUPPORT_INTERFACE::POSITION);
      support->setModeX(2, mwoibn::SUPPORT_INTERFACE::POSITION);
      support->setModeX(3, mwoibn::SUPPORT_INTERFACE::POSITION);
      support->setModeY(0, mwoibn::SUPPORT_INTERFACE::POSITION);
      support->setModeY(1, mwoibn::SUPPORT_INTERFACE::POSITION);
      support->setModeY(2, mwoibn::SUPPORT_INTERFACE::POSITION);
      support->setModeY(3, mwoibn::SUPPORT_INTERFACE::POSITION);
      res.success = true;
      return true;
    }
    else if (req.d == 3)
    {
      support->setStep(req.nr / 10000.0);
    }
    else if (req.d == 4)
    {
      support->setUpperLimit(req.nr / 100.0);
    }
    else if (req.d == 5)
    {
      support->setLowerLimit(req.nr / 100.0);
    }
    else if (req.d == 6)
    {
      support->setRadious(req.nr / 100.0);
    }
    else
    {
      res.success = false;
      return false;
    }
    res.success = true;
    return true;
  }
  else if (req.p == 4)
  { // base
    if (req.d > 3 || req.d < 0)
    {
      res.success = false;
      return false;
    }
    controller->setSteering(req.d, req.nr * 3.14 / 180);
  }
  else if (req.p == 5)
  { // base
    if (req.d > 3 || req.d < 0)
    {
      res.success = false;
      return false;
    }
    controller->setCamber(req.d, req.nr * 3.14 / 180 / 10);
  }
  else if (req.p == 6)
  { // base
    if (req.d > 3 || req.d < 0)
    {
      res.success = false;
      return false;
    }
    controller->setCastor(req.d, req.nr * 3.14 / 180 / 10);
  }
  else if (req.p == 7)
  {
    if (req.d > 3 || req.d < 0)
    {
      res.success = false;
      return false;
    }
    support->setModeX(req.d, mwoibn::SUPPORT_INTERFACE::VELOCITY);
    support->setVelX(req.d, req.nr / 1000.0);
  }
  else if (req.p == 8)
  {
    if (req.d > 3 || req.d < 0)
    {
      res.success = false;
      return false;
    }
    support->setModeY(req.d, mwoibn::SUPPORT_INTERFACE::VELOCITY);
    support->setVelY(req.d, req.nr / 1000.0);
  }
  else if (req.p == 9)
  {
    if (req.d > 3 || req.d < 0)
    {
      res.success = false;
      return false;
    }
    controller->claim(req.d);
    support->setModeZ(req.d, mwoibn::SUPPORT_INTERFACE::VELOCITY);
    support->setVelZ(req.d, req.nr / 1000.0);
  }
  else if (req.p == 10)
  {
    if (req.d > 3 || req.d < 0)
    {
      res.success = false;
      return false;
    }
    support->setVelX(req.d, 0.0);
    support->setModeX(req.d, mwoibn::SUPPORT_INTERFACE::POSITION);
  }
  else if (req.p == 11)
  {
    if (req.d > 3 || req.d < 0)
    {
      res.success = false;
      return false;
    }
    support->setVelY(req.d, 0.0);
    support->setModeY(req.d, mwoibn::SUPPORT_INTERFACE::POSITION);
  }
  else if (req.p == 12)
  {
    if (req.d > 3 || req.d < 0)
    {
      res.success = false;
      return false;
    }
    controller->release(req.d);
    support->setVelZ(req.d, 0.0);
    support->setModeZ(req.d, mwoibn::SUPPORT_INTERFACE::POSITION);
  }
  else if (req.p == 13)
  {
    if (req.d > 3 || req.d < 0)
    {
      res.success = false;
      return false;
    }
    support->setDesX(req.d, req.nr / 100.0);
    support->setModeX(req.d, mwoibn::SUPPORT_INTERFACE::POSITION);
  }
  else if (req.p == 14)
  {
    if (req.d > 3 || req.d < 0)
    {
      res.success = false;
      return false;
    }
    support->setDesY(req.d, req.nr / 100.0);
    support->setModeY(req.d, mwoibn::SUPPORT_INTERFACE::POSITION);
  }
  else if (req.p == 15)
  {
    if (req.d > 3 || req.d < 0)
    {
      res.success = false;
      return false;
    }
    support->setDesZ(req.d, req.nr / 100.0);
    support->setModeZ(req.d, mwoibn::SUPPORT_INTERFACE::POSITION);
  }
  else
  {
    res.success = false;
    return false;
  }

  res.success = true;
  return true;
}
