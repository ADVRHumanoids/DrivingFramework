#ifndef __MGNSS_ROS_PLUGINS_ODOMETRY_2_H
#define __MGNSS_ROS_PLUGINS_ODOMETRY_2_H

#include "mgnss/plugins/generator.h"
#include "mgnss/state_estimation/odometry_v2.h"

namespace mgnss
{
namespace nrt_software {
namespace plugins
{

  template<typename Subscriber, typename Service, typename Node, typename Publisher>
  class Odometry2 : public mgnss::plugins::Generator<Subscriber, Service, Node, Publisher>
  {
    typedef mgnss::plugins::Generator<Subscriber, Service, Node, Publisher> Generator_;


public:
Odometry2() : Generator_("odometry") {}

virtual ~Odometry2(){
}


protected:

virtual void _resetPrt(YAML::Node config){
        Generator_::controller_ptr.reset(new mgnss::state_estimation::OdometryV2(*Generator_::_robot_ptr.begin()->second, config));
}

virtual void _initCallbacks(YAML::Node config){
}

};
}
}
}
#endif // RT_MY_TEST_H
