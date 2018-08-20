#ifndef __MGNSS_XBOT_PLUGINS_WHEELED_MOTION_WORLD_H
#define __MGNSS_XBOT_PLUGINS_WHEELED_MOTION_WORLD_H


#include "mgnss/xbot_plugins/wheels_controllers.h"
#include "mgnss/controllers/wheeled_motion_world.h"

namespace mgnss
{
namespace xbot_plugins
{

class WheeledMotionWorld : public WheelsControllerExtend {
//  public:
//    WheeledMotionWorld(int argc, char** argv): WheelsControllerExtend(argc, argv){}
//    ~WheeledMotionWorld(){}

protected:
virtual void _resetPrt(YAML::Node config){
        _controller_ptr.reset(new mgnss::controllers::WheeledMotionWorld(*_robot_ptr, config));
}
};

}
}
#endif // RT_MY_TEST_H
