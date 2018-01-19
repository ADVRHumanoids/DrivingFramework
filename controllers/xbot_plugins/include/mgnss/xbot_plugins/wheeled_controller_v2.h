#ifndef __MGNSS_XBOT_PLUGINS_WHEELED_CONTROLLER_V2_H
#define __MGNSS_XBOT_PLUGINS_WHEELED_CONTROLLER_V2_H

#include <XCM/XBotControlPlugin.h>

#include <mwoibn/robot_class/robot_xbot_rt.h>
#include <mgnss/controllers/wheeled_motion_event.h>

#include <custom_services/updatePDGains.h>

namespace mgnss
{
namespace xbot_plugins
{
class WheelsV2 : public XBot::XBotControlPlugin
{

public:
  virtual bool init_control_plugin(XBot::Handle::Ptr handle);

  virtual bool close();

  virtual void on_start(double time);

  virtual void on_stop(double time);

protected:
  virtual void control_loop(double time, double period);

private:
  bool evenstHandler(custom_services::updatePDGains::Request& req,
                     custom_services::updatePDGains::Response& res){

        if (req.p == 1)
        { // base
          if (req.d == 1)
            _controller_ptr->setBaseDotX(req.nr / 100.0);
          else if (req.d == 2)
            _controller_ptr->setBaseDotY(req.nr / 100.0);
          else if (req.d == 3)
            _controller_ptr->setBaseDotZ(req.nr / 100.0);
          else if (req.d == 4)
          {
            _controller_ptr->setBaseDotHeading(req.nr / 100.0);
          }
          else if (req.d == 5)
            _controller_ptr->setBaseRotVelX(req.nr / 100.0);
          else if (req.d == 6)
            _controller_ptr->setBaseRotVelY(req.nr / 100.0);
          else
          {
            res.success = false;
            return false;
          }
          res.success = true;
          return true;
        }
        else if (req.p == 2)
        { // base
          if (req.d == 1)
            _controller_ptr->setBaseX(req.nr / 100.0);
          else if (req.d == 2)
            _controller_ptr->setBaseY(req.nr / 100.0);
          else if (req.d == 3)
            _controller_ptr->setBaseZ(req.nr / 100.0);
          else if (req.d == 4)
            _controller_ptr->setBaseHeading(req.nr / 100.0);
          else if (req.d == 5)
            _controller_ptr->rotateBaseX(req.nr / 100.0);
          else if (req.d == 6)
            _controller_ptr->rotateBaseY(req.nr / 100.0);
          else
          {
            res.success = false;
            return false;
          }
          res.success = true;
          return true;
        }
        return false;
    }
    
    
  bool _initialized = false, _valid = false, _rate = false;
  std::unique_ptr<mwoibn::robot_class::Robot> _robot_ptr;
  std::unique_ptr<mwoibn::WheeledMotionEvent> _controller_ptr;
  XBot::RosUtils::ServiceServerWrapper::Ptr _srv_rt;
  Eigen::Matrix<double, 12, 1> _support;

};
}
}
#endif // RT_MY_TEST_H
