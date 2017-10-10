#include <mgnss/controllers/wheeled_motion.h>
#include <mgnss/controllers/wheeled_references.h>
#include <custom_services/updatePDGains.h>
#include <mwoibn/robot_class/robot_ros_nrt.h>
#include <mwoibn/robot_class/robot_xbot_nrt.h>

bool evenstHandler(custom_services::updatePDGains::Request& req,
                   custom_services::updatePDGains::Response& res,
                   mwoibn::SupportPolygon* support, mwoibn::Base* base);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wheels_reference"); // initalize node

  ros::NodeHandle n;

  // init wheels_controller

  mwoibn::robot_class::RobotXBotNRT robot(
        "/home/user/malgorzata/workspace/src/locomotion_framework/configs/"
        "mwoibn_v2.yaml",
        "higher_scheme");

  mwoibn::WheeledMotion wheeld_controller(robot);

  mwoibn::SupportPolygon support(0.4, 0.22);
  support.setBase(0.225, 0.125);             // HARDCODED
  support.setUpperLimit(-10 * 3.1416 / 180);  
  support.setLowerLimit(-80 * 3.1416 / 180); 
  support.setRadious(0.38);                  

  mwoibn::Base base;

  base.heading.setUpperLimit(2 * 3.1416 / 180);  
  base.heading.setLowerLimit(-2 * 3.1416 / 180);

  // ros topics/service support
  ros::ServiceServer service =
      n.advertiseService<custom_services::updatePDGains::Request,
                         custom_services::updatePDGains::Response>(
          "wheels_command",
          boost::bind(&evenstHandler, _1, _2, &support, &base));

  // starting
  base.setBasePosition(wheeld_controller.getBodyPosition());

  base.pose.setCurrent(wheeld_controller.getBodyPosition());
  base.height.setCurrent(0.45);
  base.heading.setCurrent(0);
  support.setCurrent(wheeld_controller.getSupportReference());

  support.initMotion(mwoibn::SUPPORT_MOTION::DIRECT,
                     mwoibn::SUPPORT_STATE::DEFAULT);
  base.initMotion(mwoibn::BASE_MOTION::STOP, mwoibn::BASE_DIRECTION::POSITIVE);

  while (wheeld_controller.isRunning())
  {
    support.update();
    base.update();

    wheeld_controller.fullUpdate(support.get(), base.getPosition(),
                                 base.heading.get()[0]);

  }
}

bool evenstHandler(custom_services::updatePDGains::Request& req,
                   custom_services::updatePDGains::Response& res,
                   mwoibn::SupportPolygon* support, mwoibn::Base* base)
{
  bool correct = true;
  if (req.p == 1) // base
  {
    mwoibn::BASE_MOTION motion;
    mwoibn::BASE_DIRECTION direction;
    if (req.d > 0 && req.d < 3)
    {
      motion = static_cast<mwoibn::BASE_MOTION>(req.d);

      if (req.nr > 1 || req.nr < 0)
        correct = false;
      else
        direction = static_cast<mwoibn::BASE_DIRECTION>(req.nr);
      if (correct)
        base->initMotion(motion, direction);
    }
    else if (req.d == 3)
    {
      base->heading.setStep(req.nr / 10000.0);
      base->pose.setStep(req.nr / 10000.0);
    }
    else if (req.d == 4)
    {
      base->heading.setUpperLimit(req.nr / 100.0);
    }
    else if (req.d == 5)
    {
      base->heading.setLowerLimit(req.nr / 100.0);
    }
    else if (req.d == 6)
    {
      base->pose.setRadious(req.nr / 100.0);
    }
    else if (req.d == 7)
    {
      base->height.setCurrent(req.nr / 100.0);
    }
    else
      correct = false;
  }
  else if (req.p == 2) // support
  {
    mwoibn::SUPPORT_MOTION motion;
    mwoibn::SUPPORT_STATE state;
    if (req.d > 0 && req.d < 3)
    {
      motion = static_cast<mwoibn::SUPPORT_MOTION>(req.d);

      if (req.nr > 2 || req.nr < 0)
        correct = false;
      else
        state = static_cast<mwoibn::SUPPORT_STATE>(req.nr);
      if (correct)
        support->initMotion(motion, state);
    }
        else if (req.d == 3){
          support->setStep(req.nr/10000.0);
        }
        else if (req.d == 4){
          support->setUpperLimit(req.nr/100.0);
        }
        else if (req.d == 5){
          support->setLowerLimit(req.nr/100.0);
        }
        else if (req.d == 6){
          support->setRadious(req.nr/100.0);
        }
    else correct = false;
  }
  else
    correct = false;

  res.success = correct;
  return correct;
}
