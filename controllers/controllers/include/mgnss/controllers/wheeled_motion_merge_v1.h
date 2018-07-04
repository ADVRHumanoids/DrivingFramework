#ifndef __MGNSS_CONTROLLERS__WHEELED_MOTION_MERGE_V1_H
#define __MGNSS_CONTROLLERS__WHEELED_MOTION_MERGE_V1_H

#include "mgnss/controllers/wheeled_motion_actions.h"

// #include <mwoibn/hierarchical_control/tasks/center_of_mass_task.h>
// #include <mwoibn/hierarchical_control/controllers/actions.h>

#include <mwoibn/hierarchical_control/actions/merge.h>
#include <mwoibn/hierarchical_control/actions/compute.h>

namespace mgnss
{
namespace controllers
{

class WheeledMotionMergeV1 : public WheeledMotionActions
{

public:
WheeledMotionMergeV1(mwoibn::robot_class::Robot& robot, std::string config_file);
WheeledMotionMergeV1(mwoibn::robot_class::Robot& robot, YAML::Node config);

virtual ~WheeledMotionMergeV1() {
}

virtual void switchToCastor(int i, double mu){
        std::cout << "switchToCastor" << std::endl;
        _leg_merge_ptr->secondary(i);
}

virtual void switchToCamber(int i, double mu){
        std::cout << "switchToCamber" << std::endl;
        _leg_merge_ptr->primary(i);
}


protected:

std::unique_ptr<mwoibn::hierarchical_control::actions::Merge> _leg_merge_ptr;
std::unique_ptr<mwoibn::hierarchical_control::actions::Compute> _leg_camber_action;
std::unique_ptr<mwoibn::hierarchical_control::actions::Compute> _leg_castor_action;

virtual void _createAngleTasks(YAML::Node config);
virtual void _initIK(YAML::Node config);
virtual void _init(YAML::Node config);


};
}
}

#endif // WHEELED_MOTION_H
