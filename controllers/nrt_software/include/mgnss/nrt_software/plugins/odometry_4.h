#ifndef __MGNSS_ROS_PLUGINS_ODOMETRY_4_H
#define __MGNSS_ROS_PLUGINS_ODOMETRY_4_H

#include "mgnss/plugins/generator.h"
#include "mgnss/state_estimation/odometry_v4.h"

namespace mgnss
{
namespace nrt_software {
namespace plugins
{

  template<typename Subscriber, typename Service, typename Node>
  class Odometry4 : public mgnss::plugins::Generator<Subscriber, Service, Node>
  {
    typedef mgnss::plugins::Generator<Subscriber, Service, Node> Generator_;


public:
Odometry4() : Generator_("odometry") {}

virtual ~Odometry4(){
}


protected:

virtual void _resetPrt(YAML::Node config){
        Generator_::controller_ptr.reset(new mgnss::state_estimation::OdometryV4(*Generator_::_robot_ptr.begin()->second, config));
}

virtual void _initCallbacks(YAML::Node config){
}

};
}
}
}
#endif // RT_MY_TEST_H
