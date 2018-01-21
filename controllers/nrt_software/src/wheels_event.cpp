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

  mwoibn::robot_class::Robot& robot =
      loader.init(std::string(DRIVING_FRAMEWORK_WORKSPACE) + "DrivingFramework/"
                  "locomotion_framework/configs/mwoibn_v2.yaml",
                  "default", std::string(DRIVING_FRAMEWORK_WORKSPACE) + "DrivingFramework/"
                  "locomotion_framework/configs/lower_body.yaml");
#ifdef FULL_ROBOT
  mwoibn::WheeledMotionEvent wheeld_controller(robot);
#endif
#ifdef TWO_ROBOTS
  mwoibn::robot_class::Robot& full_robot = loader_2.init(
      std::string(DRIVING_FRAMEWORK_WORKSPACE) + "DrivingFramework/"
      "locomotion_framework/configs/mwoibn_v2.yaml",
      "full_model");

//  mwoibn::robot_class::Robot& full_robot = robot;
  robot.addBiMap(full_robot, "full_body");

  mwoibn::WheeledMotionEvent2 wheeld_controller(robot, full_robot);
#endif

  mwoibn::SupportPolygon3 support(0.45, 0.22, 0.078, robot.rate());

  support.setBase(0.25, 0.125); // HARDCODED

  support.setUpperLimit(-10 * 3.1416 / 180);

  support.setLowerLimit(-80 * 3.1416 / 180);
  support.setRadious(0.38);
  support.setStep(0.0005);

  // ros topics/service support
  ros::ServiceServer service =
      n.advertiseService<custom_services::updatePDGains::Request,
                         custom_services::updatePDGains::Response>(
          "wheels_command",
          boost::bind(&evenstHandler, _1, _2, &support, &wheeld_controller));

  // starting
  support.setCurrent(wheeld_controller.getSupportReference());
  support.setDesired(wheeld_controller.getSupportReference());

  // LOG
  std::ostringstream oss;
  std::ofstream file;

  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);

  oss << "wheels_log_" << std::put_time(&tm, "%d-%m-%Y %H-%M-%S") << ".txt";

  file.open(oss.str(),  std::ios::out);

  file << "com_x\t" << "com_y\t" << "e_com_x\t" << "e_com_y\t" << "z\t" << "e_z\t";
  file << "cp_1_x\t" << "cp_1_y\t" << "cp_1_z\t";
  file << "cp_2_x\t" << "cp_2_y\t" << "cp_2_z\t";
  file << "cp_3_x\t" << "cp_3_y\t" << "cp_3_z\t";
  file << "cp_4_x\t" << "cp_4_y\t" << "cp_4_z\t";
  file << "e_cp_1_x\t" << "e_cp_1_y\t" << "e_cp_1_z\t";
  file << "e_cp_2_x\t" << "e_cp_2_y\t" << "e_cp_2_z\t";
  file << "e_cp_3_x\t" << "e_cp_3_y\t" << "e_cp_3_z\t";
  file << "e_cp_4_x\t" << "e_cp_4_y\t" << "e_cp_4_z\t";
  file << "st_1\t" << "st_2\t" << "st_3\t" << "st_4_x\t";
  file << "e_st_1\t" << "e_st_2\t" << "e_st_3\t" << "e_st_4_x\n";

  file.flush();

  while (ros::ok())
  {
    support.update();
    wheeld_controller.fullUpdate(support.get());
//    file << wheeld_controller.getCom() << "\t";
//    file << wheeld_controller.errorCom() << "\t";
//    file << wheeld_controller.getCp(0) << wheeld_controller.getCp(1) << wheeld_controller.getCp(2) << wheeld_controller.getCp(3) << "\t";
//    file << wheeld_controller.errorCp() << "\t";
//    file << wheeld_controller.getSteer() << "\t";
//    file << wheeld_controller.errorSteer() << "\n";
  }

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

    if (req.d > 0 && req.d < 3)
    {
      motion = static_cast<mwoibn::SUPPORT_MOTION>(req.d);

      if (req.nr > 0 && req.nr < 4)
      {
        state = static_cast<mwoibn::SUPPORT_STATE>(req.nr);
        support->initMotion(motion, state);
      }
      else
        support->changeMotion(motion);
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
