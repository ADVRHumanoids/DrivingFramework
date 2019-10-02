#ifndef __MGNSS_ROS_PLUGINS_GRAVITY_COMPENSATION_H
#define __MGNSS_ROS_PLUGINS_GRAVITY_COMPENSATION_H

#include "mgnss/plugins/generator.h"
#include "mgnss/controllers/centralized_controller.h"
#include "mgnss/ros_callbacks/centralized.h"
#include <mwoibn/loaders/robot.h>

namespace mgnss {
namespace nrt_software {
namespace plugins {


  template<typename Subscriber, typename Service, typename Node, typename Publisher>
  class GravityCompensation : public mgnss::plugins::Generator<Subscriber, Service, Node, Publisher>
  {
    typedef mgnss::plugins::Generator<Subscriber, Service, Node, Publisher> Generator_;


    public:
    GravityCompensation() : Generator_("gravity_compensation"){}

    mgnss::controllers::CentralizedController& get(){
            return static_cast<mgnss::controllers::CentralizedController&>(*Generator_::controller_ptr);
    }
    virtual ~GravityCompensation(){
    }


  protected:
  //ros::ServiceServer _srv_reset, _srv_switch;
  //ros::Subscriber _sub_rt;
  //std::shared_ptr<mwoibn::robot_class::Robot> _reference_ptr;


  virtual void _resetPrt(YAML::Node config){
          Generator_::controller_ptr.reset(new mgnss::controllers::CentralizedController(*Generator_::_robot_ptr.begin()->second, config));
  }

  virtual void _initCallbacks(YAML::Node config){
  /*        _sub_rt = n.subscribe<custom_messages::CustomCmnd>(
                  "CoM_regulator/contacts", 1,
                  boost::bind(&getContacts, _1, &_robot, &_robot));
   */
          //  // debbuging purposes, switch between motor side and link side reference
  /*        _srv_switch = n.advertiseService<std_srvs::SetBool::Request,
                                           std_srvs::SetBool::Response>(
                  "centralized_controller/motor_side_reference",
                  boost::bind(&setMotorSideReference, _1, _2, &_robot, &_robot,
                              &motor_side));
   */
          // debbuging purposes, set current robot position as new reference
  /*        _srv_reset = n.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(
                  "centralized_controller/reset_reference",
                  boost::bind(&resetReference, _1, _2, &_robot, &_robot,
                              &motor_side));
   */
  }

};

}
}
}
#endif // RT_MY_TEST_H
