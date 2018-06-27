#include "mgnss/nrt_software/plugins/odometry.h"

int main(int argc, char** argv)
{

        mgnss::nrt_software::plugins::Odometry controller(argc, argv);

        controller.init();
        controller.start(ros::Time::now().toSec());


        //  for(int i = 0; i < 2500; i++)
        while (ros::ok())
        {
                controller.control_loop(ros::Time::now().toSec());

/*    now = ros::Time::now().toSec();
    print.setZero();
    print[0] =  now - start;
    print.segment<6>(1) =  odometry.getRaw();
    print.segment<6>(7) =  odometry.getFiltered();

    file << print.transpose().format(fmt) << "\n";
 */
        }

        controller.close();
//  file.flush();
//  file.close();

}
