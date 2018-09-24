#include "mgnss/nrt_software/plugins/centralized.h"

int main(int argc, char** argv)
{

        mgnss::nrt_software::plugins::Centralized controller(argc, argv);

        controller.init();
        controller.start(ros::Time::now().toSec());

        while(ros::ok()) {
                controller.control_loop(ros::Time::now().toSec());
        }

        controller.close();
}