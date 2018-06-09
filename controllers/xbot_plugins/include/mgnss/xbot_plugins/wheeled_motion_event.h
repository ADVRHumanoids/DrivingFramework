#ifndef __MGNSS_XBOT_PLUGINS_WHEELED_MOTION_EVENT_H
#define __MGNSS_XBOT_PLUGINS_WHEELED_MOTION_EVENT_H

#include <mgnss/controllers/wheeled_motion_event.h>
#include <mgnss/xbot_plugins/wheels_controllers.h>


namespace mgnss
{
namespace xbot_plugins
{

class WheeledMotionEvent : public WheelsControllerExtend
{
//  public:
//    WheeledMotionEvent(int argc, char** argv): WheelsControllerExtend(argc, argv){}
//    ~WheeledMotionEvent(){}

  protected:
      virtual void _resetPrt(YAML::Node config){
        _controller_ptr.reset(new mgnss::controllers::WheeledMotionEvent(*_robot_ptr, config));
      }
};

}
}
#endif // RT_MY_TEST_H
