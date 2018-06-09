#ifndef __MGNSS_XBOT_PLUGINS_WHEELED_MOTION_EVENT_3_H
#define __MGNSS_XBOT_PLUGINS_WHEELED_MOTION_EVENT_3_H

#include <mgnss/controllers/wheeled_motion_event_v3.h>
#include <mgnss/xbot_plugins/wheels_controllers.h>


namespace mgnss
{
namespace xbot_plugins
{

class WheeledMotionEvent3 : public WheelsControllerExtend{
//  public:
//    WheeledMotionEvent3(int argc, char** argv): WheelsControllerExtend(argc, argv){}
//    ~WheeledMotionEvent3(){}

  protected:
      virtual void _resetPrt(YAML::Node config){
        _controller_ptr.reset(new mgnss::controllers::WheeledMotionEvent3(*_robot_ptr, config));
      }
};

}
}
#endif // RT_MY_TEST_H
