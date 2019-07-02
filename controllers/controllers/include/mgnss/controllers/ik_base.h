#ifndef __MGNSS_CONTROLLERS_IK_BASE_H
#define __MGNSS_CONTROLLERS_IK_BASE_H

#include "mgnss/modules/base.h"

#include <mwoibn/hierarchical_control/controllers/basic.h>

#include <mwoibn/hierarchical_control/controllers/actions.h>
#include <mwoibn/hierarchical_control/actions/task.h>
#include <mwoibn/hierarchical_control/actions/compute.h>

#include <mgnss/higher_level/qp/qp_action.h>
#include <mgnss/higher_level/qp/tasks/qp_aggravated.h>

#include <mgnss/higher_level/qp/tasks/qr_task_wrapper.h>

#include <chrono>
namespace mgnss
{

namespace controllers {

class IKBase : public mgnss::modules::Base
{

public:
IKBase(mwoibn::robot_class::Robot& robot);

virtual ~IKBase() {}

virtual void init(){
        _robot.wait();
        _robot.get();
        _robot.updateKinematics();
        _robot.centerOfMass().update();

        _setInitialConditions();
}


virtual void log(mwoibn::common::Logger& logger, double time){
}

virtual void stop(){
        _command.setZero();
        _robot.command.velocity.set(_command);
        _robot.send();
}

virtual void send(){
        _robot.send();
}

virtual void close(){}

virtual void setRate(double rate){
        mgnss::modules::Base::setRate(rate);
        setRate();
}

virtual void update(){
        step();
        compute();
}

virtual void setRate(){
        _dt = _robot.rate();
}


virtual void step() = 0;


virtual void compute();


virtual bool isRunning() {
        return _robot.isRunning();
}


// virtual void nextStep() = 0;


protected:

// std::unique_ptr<mwoibn::hierarchical_control::tasks::Constraints> _constraints_ptr;
//
// std::unique_ptr<mwoibn::hierarchical_control::tasks::CartesianSelective> _pelvis_position_ptr;
// std::unique_ptr<mwoibn::hierarchical_control::tasks::OrientationSelective> _pelvis_orientation_ptr;
//
// std::unique_ptr<mwoibn::hierarchical_control::tasks::ContactPoint> _steering_ptr;

// std::vector<std::unique_ptr<mgnss::higher_level::QrTaskWrapper> > _qr_wrappers;
std::map<std::string, std::unique_ptr<mgnss::higher_level::QrTask> > _qr_wrappers;
std::vector<std::unique_ptr<mgnss::higher_level::QpAggravated> > _qp_aggravated;


std::unique_ptr<mwoibn::hierarchical_control::controllers::Actions> _ik_ptr;

std::map<std::string, mwoibn::hierarchical_control::tasks::BasicTask*> _tasks;  // Adding that only helps with automatic IK generation
std::map<std::string, std::shared_ptr<mwoibn::hierarchical_control::actions::Task> > _actions;  // Adding that only helps with automatic IK generation


//double rate = 200;
double _dt;
mwoibn::VectorN _command;
//mwoibn::Vector3 _position, _next_step;
mwoibn::Axis _x, _y, _z;

mwoibn::VectorN _active_state;
std::vector<std::string> _log_names;
mwoibn::VectorInt _select_ik;

void _create(YAML::Node config);

virtual void _setInitialConditions() = 0;
virtual void _allocate();
virtual void _createTasks(YAML::Node config) = 0;
virtual mwoibn::hierarchical_control::actions::Task& _createAction(std::string task, YAML::Node config, YAML::Node full_config);

virtual std::shared_ptr<mwoibn::hierarchical_control::actions::Task> _taskAction(std::string task, YAML::Node config, std::string type, YAML::Node full_config);
virtual double _readTask(YAML::Node config, std::string task, mwoibn::VectorN& gain);

virtual void _initIK(YAML::Node config);
//virtual void _initSteering(YAML::Node config, std::function<>);


};
}
}
#endif // WHEELED_MOTION_H
