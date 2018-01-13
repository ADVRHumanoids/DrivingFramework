#ifndef PROGRAM_RT_MY_TEST_H
#define PROGRAM_RT_MY_TEST_H

#include <XCM/XBotControlPlugin.h>
#include <mwoibn/robot_class/robot_xbot_rt.h>

namespace mgnss {

namespace xbot_plugins{

class MyTest : public XBot::XBotControlPlugin{


public:

virtual bool init_control_plugin(std::string path_to_config_file,
                             XBot::SharedMemory::Ptr shared_memory,
                             XBot::RobotInterface::Ptr robot);

virtual bool close();

virtual void on_start(double time);

virtual void on_stop(double time);

protected:

    virtual void control_loop(double time, double period);

private:

  std::unique_ptr<mwoibn::robot_class::Robot> _robot_ptr;


  double _start_time;
  mwoibn::VectorN command;

};
}
}
#endif // RT_MY_TEST_H
