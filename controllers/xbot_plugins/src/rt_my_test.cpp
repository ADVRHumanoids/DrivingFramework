#include "mgnss/xbot_plugins/rt_my_test.h"


REGISTER_XBOT_PLUGIN(MyTest, mgnss::xbot_plugins::MyTest)


bool mgnss::xbot_plugins::MyTest::init_control_plugin(XBot::Handle::Ptr handle)
{
        /* This function is called outside the real time loop, so we can
         * allocate memory on the heap, print stuff, ...
         * The RT plugin will be executed only if this init function returns true. */

        /* Save robot to a private member. */

        _robot_ptr.reset(new mwoibn::robot_class::RobotXBotRT(handle->getRobotInterface(), handle->getPathToConfigFile(), "robot3", handle->getSharedMemory()));

        command.resize(_robot_ptr->getDofs());

        command << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.14 / 2, 3.14 / 4, 3.14 / 4, 0, 0,
        0, -3.14 / 2, -3.14 / 4, -3.14 / 4, 0, 0, 0, -3.14 / 2, -3.14 / 4,
        -3.14 / 4, 0, 0, 0, 3.14 / 2, 3.14 / 4, 3.14 / 4, 0, 0, 0, 0, 0.0,
        -0.5236, -0.5236, -0.7854, 0.0, -0.7854, 0.0, 0.0, 0.5236, 0.5236, 0.7854,
        0.0, 0.7854, 0.0, 0.0, 0.0, 0.0;

////      command << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
////          0, -1.57, -2.41, -0.90, 0, 0,
////          0,  1.57,  2.41,  0.90, 0, 0,
////          0,  1.57,  2.41,  0.90, 0, 0,
////          0, -1.57, -2.41, -0.90, 0, 0,
////          0.0, 0.0, -0.5236, -0.5236, -0.7854, 0.0, -0.7854,
////          0.0, 0.0,  0.5236,  0.5236,  0.7854, 0.0, 0.7854, 0.0;
//        command << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//            0, -1.57, -2.41, -0.90, 0, 0,
//            0,  1.57,  2.41,  0.90, 0, 0,
//            0,  1.57,  2.41,  0.90, 0, 0,
//            0, -1.57, -2.41, -0.90, 0, 0;

        _robot_ptr->command.position.set(command);

        return true;

}

void mgnss::xbot_plugins::MyTest::on_start(double time)
{

}

void mgnss::xbot_plugins::MyTest::on_stop(double time)
{
}


void mgnss::xbot_plugins::MyTest::control_loop(double time)
{
}

bool mgnss::xbot_plugins::MyTest::close()
{
        return true;
}
