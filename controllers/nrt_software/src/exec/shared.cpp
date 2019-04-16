#include "mgnss/plugins/ros_shared.h"
#include <config.h>

int main(int argc, char** argv)
{
        mgnss::plugins::RosShared controller;
        controller.connect(argc, argv, "shared");

        controller.init();
        controller.start(ros::Time::now().toSec());

        while (ros::ok())
        // for(int i =0; i < 25000; i++)
        {
            // if(i%100 == 0)
                // std::cout << (i+1) << "/" << 10000 << std::endl;
                controller.control_loop(ros::Time::now().toSec());
        }

        controller.stop();
        controller.close();

}
