#include "mgnss/xbot_plugins/gravity_test.h"

REGISTER_XBOT_PLUGIN(GravityTest, mgnss::xbot_plugins::GravityTest)

bool mgnss::xbot_plugins::GravityTest::init_control_plugin(std::string path_to_config_file,
                                                           XBot::SharedMemory::Ptr shared_memory,
                                                           XBot::RobotInterface::Ptr robot)
{
        /* Save robot to a private member. */
        _robot_ptr.reset(new mwoibn::robot_class::RobotXBotRT(
                                 path_to_config_file, "higher_scheme", shared_memory));

        _command = mwoibn::VectorN::Zero(_robot_ptr->getDofs());

        _robot_ptr->update();

        _constraints_ptr.reset(
                new mwoibn::hierarchical_control::tasks::Constraints(*_robot_ptr));

        Eigen::Vector3d pelvis;
        pelvis << 1, 1, 1;

        mwoibn::point_handling::PositionsHandler pelvis_ph("ROOT", *_robot_ptr, _robot_ptr->getLinks("base"));
        mwoibn::point_handling::OrientationsHandler pelvis_orn("ROOT", *_robot_ptr, _robot_ptr->getLinks("base"));

        _pelvis_hight_ptr.reset(new
                                mwoibn::hierarchical_control::tasks::CartesianSelective(pelvis_ph, pelvis));

        _pelvis_orientation_ptr.reset(new
                                      mwoibn::hierarchical_control::tasks::OrientationSelective(pelvis_orn, pelvis, *_robot_ptr));

        // Set initaial HC tasks
        hierarchical_controller.addTask(*_constraints_ptr, 1, 1e-8);
        hierarchical_controller.addTask(*_pelvis_hight_ptr, 1000, 1e-8);
        hierarchical_controller.addTask(*_pelvis_orientation_ptr, 500, 1e-8);

        _sub_com_final.init("com_position");

        return true;
}

void mgnss::xbot_plugins::GravityTest::on_start(double time)
{
        _start_time = time;

        _initialized = _robot_ptr->get();

        if(_initialized) {
                _robot_ptr->updateKinematics();
                _init();
        }
}

void mgnss::xbot_plugins::GravityTest::_init() {

        Eigen::Vector3d axis;

        axis << 0, 0, 1;

        //_pelvis_orientation_ptr->resetReference();
        _com_ref = _pelvis_hight_ptr->points().getPointStateWorld(0);
        _com_final.head(3) = _com_ref;

        _pelvis_hight_ptr->setReference(_com_ref);
        _pelvis_orientation_ptr->setReference(0,_pelvis_orientation_ptr->getOffset(0)*mwoibn::Quaternion::fromAxisAngle(axis, 0.0));
        _state = _robot_ptr->state.get(mwoibn::robot_class::INTERFACE::POSITION);

        hierarchical_controller.update();
        _initialized = true;

}
void mgnss::xbot_plugins::GravityTest::_getReference() {

        _sub_com_final.read(_com_final);

        if(_com_final[3] != mwoibn::IS_VALID) return;

        for (int i = 0; i < 3; i++)
        {
                if (std::fabs(_com_final[i] - _com_ref[i]) > _eps)
                {
                        if (_com_final[i] - _com_ref[i] > 0)
                                _com_ref[i] += _eps;
                        else
                                _com_ref[i] -= _eps;
                }
                else
                        _com_ref[i] = _com_final[i];
        }

        _pelvis_hight_ptr->setReference(_com_ref);
}

void mgnss::xbot_plugins::GravityTest::on_stop(double time) {
}

void mgnss::xbot_plugins::GravityTest::control_loop(double time, double period)
{
        _valid = _robot_ptr->get();

        if(!_valid) return;
        _robot_ptr->updateKinematics();

        if(!_initialized) _init();


        _getReference();


        _command.noalias() = hierarchical_controller.update() * period;


        _command.noalias() += _robot_ptr->state.get(mwoibn::robot_class::INTERFACE::POSITION);

        _command.tail(15) = _state.tail(15); // maybe to be removed

        _robot_ptr->command.set(_command, mwoibn::robot_class::INTERFACE::POSITION);
        _robot_ptr->send();
}

bool mgnss::xbot_plugins::GravityTest::close() {
        return true;
}
