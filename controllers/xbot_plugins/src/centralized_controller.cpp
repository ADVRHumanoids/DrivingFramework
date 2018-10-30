#include "mgnss/xbot_plugins/centralized_controller.h"

REGISTER_XBOT_PLUGIN(CentralizedController, mgnss::xbot_plugins::CentralizedController)

bool mgnss::xbot_plugins::CentralizedController::init_control_plugin(
        std::string path_to_config_file, XBot::SharedMemory::Ptr shared_memory,
        XBot::RobotInterface::Ptr robot)
{
        /* This function is called outside the real time loop, so we can
         * allocate memory on the heap, print stuff, ...
         * The RT plugin will be executed only if this init function returns true. */

        /* Save robot to a private member. */
        _robot_ptr.reset(new mwoibn::robot_class::RobotXBotRT(
                                 path_to_config_file, "robot1", shared_memory));
        _robot_ref_ptr.reset(new mwoibn::robot_class::RobotXBotRT(
                                     path_to_config_file, "robot2", shared_memory));

        // OFFLINE
        //  _dynamic_model_ptr.reset(
        //      new mwoibn::dynamic_models::QrDecomposition(*_robot_ref_ptr)); //
        //      offline
        //  _gravity_compensation_ptr.reset(
        //      new mwoibn::gravity_compensation::SimpleQRGravityCompensation(
        //          *_dynamic_model_ptr, *_robot_ptr));
        //  _actuation_model_ptr.reset(new mwoibn::motor_side_reference::SeaReference(
        //      *_robot_ptr, *_robot_ref_ptr, *_gravity_compensation_ptr));

        // ONLINE
        _dynamic_model_ptr.reset(
                new mwoibn::dynamic_models::QrDecomposition(*_robot_ptr)); // offline
        _gravity_compensation_ptr.reset(
                new mwoibn::gravity_compensation::SimpleQRGravityCompensation(
                        *_dynamic_model_ptr, *_robot_ptr));

        _actuation_model_ptr.reset(new mwoibn::motor_side_reference::SeaReference(
                                           *_robot_ptr, *_gravity_compensation_ptr));

        return true;
}

void mgnss::xbot_plugins::CentralizedController::on_start(double time)
{
        _start_time = time;

        _robot_ref_ptr->update();
        _robot_ptr->update();
        _valid = _robot_ptr->feedbacks.initialized();
        _gravity_compensation_ptr->update();
        _robot_ptr->command.position.set(_robot_ref_ptr->state.position.get());
        _actuation_model_ptr->update();

}

void mgnss::xbot_plugins::CentralizedController::on_stop(double time) {
}

void mgnss::xbot_plugins::CentralizedController::control_loop(double time, double period)
{
        if(!_robot_ptr->get()) return;

        if (_robot_ref_ptr->get())
        {
                _robot_ptr->command.position.set(_robot_ref_ptr->state.position.get());
        }

        //  _robot_ref_ptr->updateKinematics(); // ONLINE LOOP
        _robot_ptr->updateKinematics();

        _gravity_compensation_ptr->update();

        if (_motor_side)
                _actuation_model_ptr->update();
//  else
//    _robot_ptr->command.position.set(_robot_ref_ptr->state.position.get());

        _robot_ptr->send();
}

bool mgnss::xbot_plugins::CentralizedController::close() {
        return true;
}
