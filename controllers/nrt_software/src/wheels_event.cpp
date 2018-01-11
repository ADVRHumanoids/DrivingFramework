#include <config.h>
#include <mwoibn/loaders/robot.h>

#include <mgnss/controllers/wheeled_motion_event_v2.h>
//#include <mgnss/controllers/wheeled_motion_event.h>
#include <mgnss/controllers/wheeled_references_v3.h>
#include <custom_services/updatePDGains.h>

bool evenstHandler(custom_services::updatePDGains::Request& req,
                   custom_services::updatePDGains::Response& res,
                   mwoibn::SupportPolygon3* support,
                   mwoibn::WheeledMotionEvent2* controller);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wheels_reference"); // initalize node

  ros::NodeHandle n;

  // init wheels_controller
  mwoibn::loaders::Robot loader, loader_2;

//  mwoibn::loaders::Robot loader;
  mwoibn::robot_class::Robot& robot =
      loader.init(std::string(DRIVING_FRAMEWORK_WORKSPACE) + "DrivingFramework/"
                  "locomotion_framework/configs/mwoibn_v2.yaml",
                  "default", std::string(DRIVING_FRAMEWORK_WORKSPACE) + "DrivingFramework/"
                  "locomotion_framework/configs/lower_body.yaml");

  mwoibn::robot_class::Robot& full_robot = loader_2.init(
      std::string(DRIVING_FRAMEWORK_WORKSPACE) + "DrivingFramework/"
      "locomotion_framework/configs/mwoibn_v2.yaml",
      "full_model");

//  mwoibn::robot_class::Robot& full_robot = robot;

  robot.addBiMap(full_robot, "full_body");
  mwoibn::WheeledMotionEvent2 wheeld_controller(robot, full_robot);
//  mwoibn::WheeledMotionEvent wheeld_controller(robot);

  mwoibn::SupportPolygon3 support(0.45, 0.22, 0.078, robot.rate());

  support.setBase(0.25, 0.125); // HARDCODED

  support.setUpperLimit(-10 * 3.1416 / 180);

  support.setLowerLimit(-80 * 3.1416 / 180);
  support.setRadious(0.38);
  support.setStep(0.0005);

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

  while (ros::ok())
  {
    support.update();
    wheeld_controller.fullUpdate(support.get());
  }
}

bool evenstHandler(custom_services::updatePDGains::Request& req,
                   custom_services::updatePDGains::Response& res,
                   mwoibn::SupportPolygon3* support,
                   mwoibn::WheeledMotionEvent2* controller)
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
