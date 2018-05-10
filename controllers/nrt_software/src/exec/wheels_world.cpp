#define FULL_ROBOT

#include <mgnss/nrt_software/plugins/wheeled_motion_world.h>
#include <config.h>
#include <mwoibn/loaders/robot.h>

#include <mgnss/controllers/wheeled_motion_world.h>
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
  mgnss::nrt_software::plugins::WheeledMotionWorld controller(argc, argv);

  controller.init();
  controller.start(ros::Time::now().toSec());

  //  for(int i = 0; i < 2500; i++)
  while (ros::ok())
  {
    controller.control_loop(ros::Time::now().toSec());
  }

  controller.stop();
  controller.close();

}
