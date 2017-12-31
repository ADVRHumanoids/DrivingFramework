#include <mwoibn/loaders/robot.h>

#include <mgnss/controllers/wheeled_motion.h>
#include <mgnss/controllers/wheeled_references.h>
#include <custom_services/updatePDGains.h>

bool evenstHandler(custom_services::updatePDGains::Request& req,
                   custom_services::updatePDGains::Response& res,
                   mwoibn::SupportPolygon* support, mwoibn::Base* base);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "wheels_reference"); // initalize node

  ros::NodeHandle n;

  // init wheels_controller
  mwoibn::loaders::Robot loader;

  mwoibn::robot_class::Robot& robot = loader.init(
        "/home/malgorzata/catkin_ws/src/DrivingFramework/locomotion_framework/configs/"
        "mwoibn_v2.yaml",
        "default");

  mwoibn::WheeledMotion wheeld_controller(robot);

  mwoibn::SupportPolygon support(0.45, 0.22);
  support.setBase(0.25, 0.125);             // HARDCODED
  support.setUpperLimit(-10 * 3.1416 / 180);
  support.setLowerLimit(-80 * 3.1416 / 180); 
  support.setRadious(0.38);
  support.setStep(0.0005);

  mwoibn::Base base;

//  base.heading.setUpperLimit(2 * 3.1416 / 180);  
//  base.heading.setLowerLimit(-2 * 3.1416 / 180);

  // ros topics/service support
  ros::ServiceServer service =
      n.advertiseService<custom_services::updatePDGains::Request,
                         custom_services::updatePDGains::Response>(
          "wheels_command",
          boost::bind(&evenstHandler, _1, _2, &support, &base));

  // starting
//  base.setBasePosition(wheeld_controller.getBodyPosition());

//  base.pose.setCurrent(wheeld_controller.getBodyPosition());

//  base.height.setCurrent(0.55); //?
//  base.heading.setCurrent(0); //?
  
  support.setCurrent(wheeld_controller.getSupportReference());

  support.initMotion(mwoibn::SUPPORT_MOTION::DIRECT,
                     mwoibn::SUPPORT_STATE::DEFAULT);
//  base.initMotion(mwoibn::BASE_MOTION::STOP, mwoibn::BASE_DIRECTION::POSITIVE);

  while (ros::ok())
  {
    support.update();
//    base.update();
//    std::cout << ros::Time::now().toSec() << "\t";
//    std::cout << base.getPosition() << std::endl;
    wheeld_controller.fullUpdate(support.get(), base.getPosition(),
                                 base.getHeading());

  }
}

bool evenstHandler(custom_services::updatePDGains::Request& req,
                   custom_services::updatePDGains::Response& res,
                   mwoibn::SupportPolygon* support, mwoibn::Base* base)
{
  bool correct = true;
  if (req.p == 1) // base
  {
     if(req.d == 1)
         base->setX(req.nr/100.0);
     else if(req.d == 2)
         base->setY(req.nr/100.0);
     else if(req.d == 3)
         base->setZ(req.nr/100.0);
     else if(req.d == 4)
         base->setHeading(req.nr/100.0);
  }
  else if (req.p == 2) // support
  {
    mwoibn::SUPPORT_MOTION motion;
    mwoibn::SUPPORT_STATE state;
    if (req.d > 0 && req.d < 3)
    {
      motion = static_cast<mwoibn::SUPPORT_MOTION>(req.d);

      if (req.nr > 3 || req.nr < 0)
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
