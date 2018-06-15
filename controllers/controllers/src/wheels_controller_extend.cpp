#include <mgnss/controllers/wheels_controller_extend.h>

mgnss::controllers::WheelsControllerExtend::WheelsControllerExtend(mwoibn::robot_class::Robot& robot)
        : WheelsController(robot)
{

}

void mgnss::controllers::WheelsControllerExtend::resetSteering()
{
        for (int i = 0; i < 4; i++)
        {
                _leg_steer_ptr->setReference(i, 0);
        }
}

void mgnss::controllers::WheelsControllerExtend::_createAngleTasks(){


        mwoibn::Axis ax;
        ax << 0, 1, 0;
        mwoibn::robot_class::angles::Caster caster1(
                _robot, mwoibn::point_handling::Point("ankle2_1", _robot.getModel()), ax);
        ax <<  0,  0,  1;
        mwoibn::robot_class::angles::Camber camber1(
                _robot, mwoibn::point_handling::Point("wheel_1", _robot.getModel()), ax);
        ax <<  0,  0,  1;
        mwoibn::robot_class::angles::Steering steer1(
                _robot, mwoibn::point_handling::Point("wheel_1", _robot.getModel()), ax);
        ax << 0, 1, 0;
        mwoibn::robot_class::angles::Caster caster3(
                _robot, mwoibn::point_handling::Point("ankle2_3", _robot.getModel()), ax);
        ax <<  0,  0,  1;
        mwoibn::robot_class::angles::Camber camber3(
                _robot, mwoibn::point_handling::Point("wheel_3", _robot.getModel()), ax);
        ax <<  0,  0,  1;
        mwoibn::robot_class::angles::Steering steer3(
                _robot, mwoibn::point_handling::Point("wheel_3", _robot.getModel()), ax);

        ax << 0, -1, 0;
        mwoibn::robot_class::angles::Caster caster2(
                _robot, mwoibn::point_handling::Point("ankle2_2", _robot.getModel()), ax);
        ax <<  0,  0,  -1;
        mwoibn::robot_class::angles::Camber camber2(
                _robot, mwoibn::point_handling::Point("wheel_2", _robot.getModel()), ax);
        ax <<  0,  0,  -1;
        mwoibn::robot_class::angles::Steering steer2(
                _robot, mwoibn::point_handling::Point("wheel_2", _robot.getModel()), ax);
        ax << 0, -1, 0;
        mwoibn::robot_class::angles::Caster caster4(
                _robot, mwoibn::point_handling::Point("ankle2_4", _robot.getModel()), ax);
        ax <<  0,  0,  -1;
        mwoibn::robot_class::angles::Camber camber4(
                _robot, mwoibn::point_handling::Point("wheel_4", _robot.getModel()), ax);
        ax <<  0,  0,  -1;
        mwoibn::robot_class::angles::Steering steer4(
                _robot, mwoibn::point_handling::Point("wheel_4", _robot.getModel()), ax);

        _leg_steer_ptr.reset(new mwoibn::hierarchical_control::tasks::SteeringAngleTask(
                                     {steer1, steer2, steer3, steer4}, _robot));
        _leg_camber_ptr.reset(new mwoibn::hierarchical_control::tasks::CamberAngleTask(
                                      {camber1, camber2, camber3, camber4}, _robot));
        _leg_castor_ptr.reset(new mwoibn::hierarchical_control::tasks::CastorAngleTask(
                                      {caster1, caster2, caster3, caster4}, _robot));

}

void mgnss::controllers::WheelsControllerExtend::_setInitialConditions(){

        _steering_ptr->init();

        _dt = _robot.rate();

        _leg_steer_ptr->updateError();
        _leg_camber_ptr->updateError();
        _leg_castor_ptr->updateError();
        _steering_ptr->updateState();

        steerings.noalias() = _leg_steer_ptr->getCurrent();
        _support.noalias() = _steering_ptr->getReference();
        _support_vel.setZero();

        _leg_steer_ptr->setReference(steerings);
        _leg_camber_ptr->setReference(_leg_camber_ptr->getCurrent());
        _leg_castor_ptr->setReference(_leg_castor_ptr->getCurrent());

}
