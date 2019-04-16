#ifndef __MGNSS_ROS_PLUGINS_COMPLIANCE_COMPENSATION_H
#define __MGNSS_ROS_PLUGINS_COMPLIANCE_COMPENSATION_H

#include "mgnss/plugins/generator.h"
#include "mgnss/controllers/compliance_compensation.h"
// #include "mgnss/ros_callbacks/centralized.h"
#include <mwoibn/loaders/robot.h>

namespace mgnss {
namespace nrt_software {
namespace plugins {


  template<typename Subscriber, typename Service, typename Node>
  class ComplianceCompensation : public mgnss::plugins::Generator<Subscriber, Service, Node>
  {
    typedef mgnss::plugins::Generator<Subscriber, Service, Node> Generator_;


    public:
    ComplianceCompensation() : Generator_("compliance_compensation"){}

    mgnss::controllers::ComplianceCompensation& get(){
            return static_cast<mgnss::controllers::ComplianceCompensation&>(*Generator_::controller_ptr);
    }
    virtual ~ComplianceCompensation(){
    }


  protected:

  virtual void _resetPrt(YAML::Node config){
          Generator_::controller_ptr.reset(new mgnss::controllers::ComplianceCompensation(*Generator_::_robot_ptr.begin()->second, config));
  }

  virtual void _initCallbacks(YAML::Node config){  }

};

}
}
}
#endif // RT_MY_TEST_H
