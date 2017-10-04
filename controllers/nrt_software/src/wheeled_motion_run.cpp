#include <mgnss/controllers/wheeled_motion.h>
#include <mgnss/controllers/wheeled_references.h>
#include <mwoibn/robot_class/robot_ros_nrt.h>

bool wait(int& flag, int max);

int main(int argc, char** argv)
{

  ros::init(argc, argv, "wheels_motion_2"); // initalize node

  ros::NodeHandle n;

  // decalare all variables
  double rate = 200, eps = 1.0 * 1e-3;
  int flag = 0;

  mwoibn::VectorN steering_error;

  mwoibn::robot_class::RobotRosNRT robot(
        "/home/user/malgorzata/workspace/src/locomotion_framework/configs/"
        "mwoibn_v2.yaml",
        "higher_scheme");

  mwoibn::WheeledMotion wheeld_controller(robot);
  mwoibn::SupportPolygon support(0.4, 0.22);
  mwoibn::Base base;

  support.setBase(0.225, 0.125);             // HARDCODED
  support.setUpperLimit(-1 * 3.1416 / 180);  // HARDCODED
  support.setLowerLimit(-80 * 3.1416 / 180); // HARDCODED
  support.setRadious(0.35);                  // HARDCODED

  base.setBasePosition(wheeld_controller.getBodyPosition());

  base.pose.setCurrent(wheeld_controller.getBodyPosition().head(2));
  base.height.setCurrent(0.42);
  base.heading.setCurrent(0);
  support.setCurrent(wheeld_controller.getSupportReference());

  std::vector<std::pair<mwoibn::SUPPORT_MOTION, mwoibn::SUPPORT_STATE>> states;
  states.push_back(std::make_pair(mwoibn::SUPPORT_MOTION::DIRECT,
                                  mwoibn::SUPPORT_STATE::MAMMAL));
  states.push_back(std::make_pair(mwoibn::SUPPORT_MOTION::CIRCULAR,
                                  mwoibn::SUPPORT_STATE::SPIDER));
  states.push_back(std::make_pair(mwoibn::SUPPORT_MOTION::CIRCULAR,
                                  mwoibn::SUPPORT_STATE::MAMMAL));

  for (auto state : states)
  {
    if (!support.initMotion(state.first, state.second))
      std::cout << "Couldn't initialized the motion" << std::endl;

    while (wheeld_controller.isRunning() && !support.update()){
      wheeld_controller.fullUpdate(support.get(), base.getPosition(),
                                   base.heading.get()[0]);
    }

    wait(flag, 200);
  }

  support.initMotion(mwoibn::SUPPORT_MOTION::DIRECT, mwoibn::SUPPORT_STATE::DEFAULT);
  base.initMotion(mwoibn::BASE_MOTION::HEADING, mwoibn::BASE_DIRECTION::NEGATIVE);

  while (wheeld_controller.isRunning() &&
         (!base.update() ||
          !wheeld_controller.isDonePlanar(25 * eps) ||
          !wheeld_controller.isDoneOrientation(100 * eps * 3.18 / 180)))
    wheeld_controller.fullUpdate(support.get(), base.getPosition(),
                                 base.heading.get()[0]);

  steering_error = mwoibn::VectorN::Ones(12);

  wheeld_controller.resetSteering();
  wheeld_controller.compute();

  while (wheeld_controller.isRunning() &&
         wheeld_controller.isDoneSteering(0.5 * 3.1416 / 180))
  {
    wheeld_controller.compute();
  }

  int wait_max = 500, mode = 0, steps = 5;
  double ds_max = 0.5 / rate, ds_step = ds_max / steps, ds = ds_step;
  bool support_done;

  base.pose.setStep(ds);
  base.heading.setStep(ds);
  support.setUpperLimit(-0.17453333);

  support.initMotion(mwoibn::SUPPORT_MOTION::DIRECT, mwoibn::SUPPORT_STATE::DEFAULT);
  base.initMotion(mwoibn::BASE_MOTION::FULL, mwoibn::BASE_DIRECTION::POSITIVE);

  while (wheeld_controller.isRunning())
  {
    support_done = support.update();
    base.update();

    // speed up
    if (mode < steps)
    {
      if (wheeld_controller.isDoneWheels(0.05))
        flag++;
      else
        flag = 0;

      if (flag > wait_max)
      {
        mode++;
        ds += ds_step;
        base.pose.setStep(ds);
        base.heading.setStep(ds);
        flag = 0;
        std::cout << "path\t" << ds << "/" << ds_max << std::endl;
      }
    }
    // wait
    else if (mode == steps)
    {
      if (wait(flag, 200))
      {
        mode++;
        support.initMotion(mwoibn::SUPPORT_MOTION::DIRECT,
                           mwoibn::SUPPORT_STATE::DEFAULT);
      }
    }

    // set correct initial support polygon
    else if (mode == steps + 1)
    {
      if (support_done)
      {
        mode++;
        flag = 0;
      }
    }
    // wait
    else if (mode == steps + 2)
    {
      if (wait(flag, wait_max))
      {
        support.initMotion(mwoibn::SUPPORT_MOTION::CIRCULAR,
                           mwoibn::SUPPORT_STATE::SPIDER);
        mode++;
      }
    }
    // circular trajectory
    else if (mode == steps + 3)
    {
      if (support_done)
      {
        support.changeDirection();
        mode++;
      }
    }
    else if (mode == steps + 4)
    {
      if (wait(flag, wait_max))
        mode = steps + 3;
    }

    wheeld_controller.fullUpdate(support.get(), base.getPosition(),
                                 base.heading.get()[0]);
  }
}

bool wait(int& flag, int max)
{

  if (flag >= max)
  {
    flag = 0;
    return true;
  }

  flag++;
  return false;
}
