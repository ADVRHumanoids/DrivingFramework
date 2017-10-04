#include <rt_plugins/rt_my_test.h>

REGISTER_XBOT_PLUGIN(RtMyTest, mwoibn::MyTest)

namespace mwoibn {

bool MyTest::init_control_plugin(std::string path_to_config_file,
        XBot::SharedMemory::Ptr shared_memory,
        XBot::RobotInterface::Ptr robot)
{
  /* This function is called outside the real time loop, so we can
   * allocate memory on the heap, print stuff, ...
   * The RT plugin will be executed only if this init function returns true. */

  /* Save robot to a private member. */
  _robot_ptr.reset(new mwoibn::robot_class::RobotXBotRT(path_to_config_file, "robot3", shared_memory));

  command.resize(_robot_ptr->getDofs());

//    command << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.14 / 2, 3.14 / 4, 3.14 / 4, 0, 0,
//        0, -3.14 / 2, -3.14 / 4, -3.14 / 4, 0, 0, 0, -3.14 / 2, -3.14 / 4,
//        -3.14 / 4, 0, 0, 0, 3.14 / 2, 3.14 / 4, 3.14 / 4, 0, 0, 0, 0, 0.0,
//        -0.5236, -0.5236, -0.7854, 0.0, -0.7854, 0.0, 0.0, 0.5236, 0.5236, 0.7854,
//        0.0, 0.7854, 0.0;

//      command << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//          0, -1.57, -2.41, -0.90, 0, 0,
//          0,  1.57,  2.41,  0.90, 0, 0,
//          0,  1.57,  2.41,  0.90, 0, 0,
//          0, -1.57, -2.41, -0.90, 0, 0,
//          0.0, 0.0, -0.5236, -0.5236, -0.7854, 0.0, -0.7854,
//          0.0, 0.0,  0.5236,  0.5236,  0.7854, 0.0, 0.7854, 0.0;
        command << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0, -1.57, -2.41, -0.90, 0, 0,
            0,  1.57,  2.41,  0.90, 0, 0,
            0,  1.57,  2.41,  0.90, 0, 0,
            0, -1.57, -2.41, -0.90, 0, 0;

  _robot_ptr->command.set(command, mwoibn::robot_class::INTERFACE::POSITION);

  return true;

}

void MyTest::on_start(double time)
{
   _start_time = time;
   _robot_ptr->update();
}

void MyTest::on_stop(double time)
{
}


void MyTest::control_loop(double time, double period)
{
  _robot_ptr->update();
}

bool MyTest::close()
{
  return true;
}

}
