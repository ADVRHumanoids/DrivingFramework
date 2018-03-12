#include <mgnss/nrt_software/plugins/joint_states.h>

int main(int argc, char** argv)
{

  mgnss::nrt_software::plugins::JointStates controller(argc, argv);

  controller.init();
  controller.start();

  while(ros::ok()){
      controller.control_loop();
  }

  controller.close();
}

