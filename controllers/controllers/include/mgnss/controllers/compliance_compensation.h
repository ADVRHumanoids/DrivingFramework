#ifndef __MGNSS_CONTROLLERS_COMPLIANCE_COMPENSATION_H
#define __MGNSS_CONTROLLERS_COMPLIANCE_COMPENSATION_H

#include "mgnss/modules/base.h"
#include <mwoibn/motor_side_reference/sea_compensation.h>

namespace mgnss {
namespace controllers {

class ComplianceCompensation : public mgnss::modules::Base {

public:
ComplianceCompensation(mwoibn::robot_class::Robot& robot, std::string config_file, std::string name);
ComplianceCompensation(mwoibn::robot_class::Robot& robot, YAML::Node config);

virtual ~ComplianceCompensation(){
}

virtual void update();

virtual void init();
virtual void send();
virtual void stop(){
}
virtual void close(){
}

virtual void initLog(mwoibn::common::Logger& logger){
        logger.addField("time", 0);
}

virtual void log(mwoibn::common::Logger& logger, double time){
        logger.addEntry("time", time);
}

protected:

virtual void _construct(YAML::Node config);
std::unique_ptr<mwoibn::motor_side_reference::SeaCompensation> _actuation_model_ptr;

//  bool _motor_side = true;
//  bool _valid = false;

};
}
}
#endif // __MGNSS_RT_PLUGINS_RT_MY_TEST_H
