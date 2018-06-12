#ifndef __MWOIBN_HIERARCHICAL_CONTROL_CONTROLLERS_CONTINOUS_H
#define __MWOIBN_HIERARCHICAL_CONTROL_CONTROLLERS_CONTINOUS_H

#include "mwoibn/hierarchical_control/controllers/default.h"
#include "mwoibn/robot_class/robot.h"

namespace mwoibn {
namespace hierarchical_control {
namespace controllers {

class Continous : public Default {
public:
Continous(mwoibn::robot_class::Robot& robot, double mu = 10000000) : Default(), _robot(robot), _mu(mu), _joint_states(robot.state.get(mwoibn::robot_class::INTERFACE::POSITION)){

								_previous_joint_states = mwoibn::VectorN::Zero(_robot.getDofs());
								__n_by_n1 = mwoibn::Matrix::Identity(_robot.getDofs(), _robot.getDofs());
								__n_by_n2 = mwoibn::Matrix::Identity(_robot.getDofs(), _robot.getDofs());
								_g_it = mwoibn::Matrix::Identity(_robot.getDofs(), _robot.getDofs());
}
~Continous(){
}
// void addTask(tasks::BasicTask& new_task, mwoibn::VectorN gain, int i = -1, double damp = 1e-8);
virtual void addTask(tasks::BasicTask& new_task, mwoibn::VectorN gain,
																					double damping = 1e-8);
virtual void addTask(tasks::BasicTask& new_task, double gain,
																					double damping = 1e-8);
virtual void addTask(tasks::BasicTask& new_task, unsigned int i, mwoibn::VectorN gain,
																					double damping = 1e-8);
virtual void addTask(tasks::BasicTask& new_task, unsigned int i, double gain,
																					double damping = 1e-8);
void removeTask(int i);

//! updates states of all of the controllers and computes the control law with compute()
/**
 * \see Basic#compute()
 *
 */
virtual const mwoibn::VectorN& update();
//! computes the controll law without updating the controllers states
/**
 * \see Basic#updateController()
 *
 */
virtual void compute();

protected:
double _last_stack_change = 0;

mwoibn::VectorN _e;
mwoibn::Matrix _g, __n_by_n1, __n_by_n2, _g_it;
mwoibn::robot_class::Robot& _robot;
void _resetCorrection();
mwoibn::VectorN _previous_joint_states;
const mwoibn::VectorN& _joint_states;
double _mu;

virtual void _resize();
virtual bool _checkStack(){
								return true;
}


};
}
} // namespace package
} // namespace library

#endif
