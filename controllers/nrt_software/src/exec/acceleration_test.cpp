#include "mwoibn/robot_class/robot.h"
#include "mwoibn/loaders/robot.h"
#include "mgnss/nrt_software/plugins/acceleration_test.h"

int main(int argc, char** argv)
{

          mgnss::nrt_software::plugins::AccelerationTest controller(argc, argv);

          controller.init();
          controller.start(ros::Time::now().toSec());

          while (ros::ok())
          {
                  controller.control_loop(ros::Time::now().toSec());
          }

          controller.close();


}
