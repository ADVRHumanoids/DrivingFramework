#include "mgnss/plugins/ros_shared.h"
#include <config.h>

int main(int argc, char** argv)
{
        mgnss::plugins::RosShared controller;
        controller.connect(argc, argv, "shared");

        controller.init();
        controller.start(ros::Time::now().toSec());

        while (ros::ok())
        {
                controller.control_loop(ros::Time::now().toSec());
        }

        controller.stop();
        controller.close();

}
