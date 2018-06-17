#define FULL_ROBOT

#include "mgnss/nrt_software/plugins/wheels_controllers.h"
#include <config.h>
#include <mwoibn/loaders/robot.h>

int main(int argc, char** argv)
{
        mgnss::nrt_software::plugins::WheeledMotionEvent controller(argc, argv);

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
