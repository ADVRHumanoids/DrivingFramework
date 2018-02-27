#ifndef __MGNSS_XBOT_PLUGINS_JOINT_STATES_H
#define __MGNSS_XBOT_PLUGINS_JOINT_STATES_H

#include <XCM/XBotControlPlugin.h>
#include <mwoibn/robot_class/robot_xbot_rt.h>
#include <mgnss/controllers/joint_states.h>
#include <custom_services/jointStateCmnd.h>

namespace mgnss
{
namespace xbot_plugins
{
class JointStates : public XBot::XBotControlPlugin
{

public:
  virtual bool init_control_plugin(XBot::Handle::Ptr handle);

  virtual bool close();

  virtual void on_start(double time);

  virtual void on_stop(double time);

protected:
  virtual void control_loop(double time, double period);

private:
  static bool referenceHandler(custom_services::jointStateCmnd::Request& req,
                    custom_services::jointStateCmnd::Response& res,
                    mgnss::controllers::JointStates* controller)
  {

      if(controller->setFullPosition(req.position)){
          res.message = "Found requested position " + req.position;
          //res.message = "Position " + req.position + " has not been defined in the robot";
      }
      else if(controller->setVelocity(req.position, req.velocity))
          res.message = "Set requested velocity for " + req.position;
      else if(controller->setPosition(req.position, req.velocity))
          res.message = "Set requested position for " + req.position;
      else
          res.message = "Unknown command " + req.position + ".";


      if(req.pos_step)
        controller->step(req.pos_step);

      res.success = true;

      return true;
    }

  std::unique_ptr<mgnss::controllers::JointStates> _controller_ptr;
  std::unique_ptr<mwoibn::robot_class::Robot> _robot_ptr;
  XBot::RosUtils::ServiceServerWrapper::Ptr _srv_rt;
  bool _initialized = false, _valid = false, _rate = false;

};
}
}
#endif // RT_MY_TEST_H
