#include "mgnss/controllers/wheels_controller.h"
#include "mgnss/higher_level/steering_v4.h"
#include <mwoibn/hierarchical_control/controllers/default.h>

mgnss::controllers::WheelsController::WheelsController(mwoibn::robot_class::Robot& robot)
        : mgnss::modules::Base(robot)
{

//  _hierarchical_controller_ptr.reset(new mwoibn::hierarchical_control::HierarchicalController());
        _hierarchical_controller_ptr.reset(new mwoibn::hierarchical_control::controllers::Default());
        _x << 1, 0, 0;
        _y << 0, 1, 0;
        _z << 0, 0, 1;
}


double mgnss::controllers::WheelsController::limit(const double th)
{
        return th - 6.28318531 * std::floor((th + 3.14159265) / 6.28318531);
}

//void mgnss::controllers::WheelsController::update(const mwoibn::VectorN& support)
//{
//  updateSupport(support);
//  nextStep();
//  compute();
//}

void mgnss::controllers::WheelsController::compute()
{
        _command.noalias() = _hierarchical_controller_ptr->update();

        _robot.command.velocity.set(_command, _select_ik);

//        std::cout << "velocity " << _command.transpose() << std::endl;
        _command.noalias() = _command * _robot.rate();
        _command.noalias() +=
                _robot.state.position.get();
//        std::cout << "position " << _command.transpose() << std::endl;
        _robot.command.position.set(_command, _select_ik);

}

void mgnss::controllers::WheelsController::step(){
        _support  += _support_vel * _robot.rate();
        _position += _linear_vel  * _robot.rate();
        _heading  += _angular_vel[2] * _robot.rate();
        _heading  -= 6.28318531 * std::floor((_heading + 3.14159265) / 6.28318531); // limit -pi:pi

}

void mgnss::controllers::WheelsController::steering()
{

        _steering_ref_ptr->compute(_next_step);

        steerings.noalias() = _steering_ref_ptr->get();

        for (int i = 0; i < 4; i++)
        {
                steerings[i] = (steerings[i] < _l_limits[i]) ? steerings[i] + mwoibn::PI : steerings[i];
                steerings[i] = (steerings[i] > _u_limits[i]) ? steerings[i] - mwoibn::PI : steerings[i];
                setSteering(i, steerings[i]);
        }
}

void mgnss::controllers::WheelsController::nextStep()
{
        _robot.centerOfMass().update();

        step();

        updateBase();
        _updateSupport();

        _next_step[0] =
                (_linear_vel[0]);
        _next_step[1] =
                (_linear_vel[1]);
        _next_step[2] =
                (_angular_vel[2]); // just limit the difference

        steering();
}

void mgnss::controllers::WheelsController::_allocate(){

        _angular_vel.setZero();
        _linear_vel.setZero();

        _select_steer = _robot.getDof(_robot.getLinks("camber"));
        steerings.setZero(_select_steer.size());

        _support.setZero(_steering_ptr->getFullTaskSize());
        _support_vel.setZero(_steering_ptr->getFullTaskSize());

        _l_limits.setZero(_select_steer.size());
        _u_limits.setZero(_select_steer.size());

        _robot.lower_limits.position.get(_l_limits, _select_steer);
        _robot.upper_limits.position.get(_u_limits, _select_steer);

        _previous_command = mwoibn::VectorN::Zero(3);
        _command.setZero(_robot.getDofs());

}
