#include "mgnss/controllers/wheels_controller_extend.h"

mgnss::controllers::WheelsControllerExtend::WheelsControllerExtend(mwoibn::robot_class::Robot& robot)
        : WheelsController(robot)
{

}

void mgnss::controllers::WheelsControllerExtend::resetSteering()
{
        for (int i = 0; i < 4; i++)
        {
                _steer_task[i].setReference(0);
        }
}

void mgnss::controllers::WheelsControllerExtend::_createAngleTasks(){


        mwoibn::Axis ax;
        ax << 0, 1, 0;
        _caster.push_back(mwoibn::robot_class::angles::Caster(_robot, mwoibn::point_handling::Frame("ankle2_1", _robot.getModel(), _robot.state), ax));
        ax << 0, -1, 0;
        _caster.push_back(mwoibn::robot_class::angles::Caster(_robot, mwoibn::point_handling::Frame("ankle2_2", _robot.getModel(), _robot.state), ax));
        ax << 0, 1, 0;
        _caster.push_back(mwoibn::robot_class::angles::Caster(_robot, mwoibn::point_handling::Frame("ankle2_3", _robot.getModel(), _robot.state), ax));
        ax << 0, -1, 0;
        _caster.push_back(mwoibn::robot_class::angles::Caster(_robot, mwoibn::point_handling::Frame("ankle2_4", _robot.getModel(), _robot.state), ax));

        ax <<  0,  0,  1;
        _camber.push_back(mwoibn::robot_class::angles::Camber( _robot, mwoibn::point_handling::Frame("wheel_1", _robot.getModel(), _robot.state), ax));
        _steer.push_back(mwoibn::robot_class::angles::Steering(_robot, mwoibn::point_handling::Frame("wheel_1", _robot.getModel(), _robot.state), ax));
        ax <<  0,  0,  -1;
        _camber.push_back(mwoibn::robot_class::angles::Camber( _robot, mwoibn::point_handling::Frame("wheel_2", _robot.getModel(), _robot.state), ax));
        _steer.push_back(mwoibn::robot_class::angles::Steering(_robot, mwoibn::point_handling::Frame("wheel_2", _robot.getModel(), _robot.state), ax));
        ax <<  0,  0,  1;
        _camber.push_back(mwoibn::robot_class::angles::Camber( _robot, mwoibn::point_handling::Frame("wheel_3", _robot.getModel(), _robot.state), ax));
        _steer.push_back(mwoibn::robot_class::angles::Steering(_robot, mwoibn::point_handling::Frame("wheel_3", _robot.getModel(), _robot.state), ax));
        ax <<  0,  0,  -1;
        _camber.push_back(mwoibn::robot_class::angles::Camber( _robot, mwoibn::point_handling::Frame("wheel_4", _robot.getModel(), _robot.state), ax));
        _steer.push_back(mwoibn::robot_class::angles::Steering(_robot, mwoibn::point_handling::Frame("wheel_4", _robot.getModel(), _robot.state), ax));

        for(auto& angle : _caster)
                _caster_task.push_back(mwoibn::hierarchical_control::tasks::Angle(angle, _robot));

        for(auto& task : _caster_task) _leg_castor.addTask(task);


        for(auto& angle : _steer)
                _steer_task.push_back(mwoibn::hierarchical_control::tasks::SoftAngle(angle, _robot));

        for(auto& task : _steer_task) _leg_steer.addTask(task);

        for(auto& angle : _camber)
                _camber_task.push_back(mwoibn::hierarchical_control::tasks::Angle(angle, _robot));

        for(auto& task : _camber_task) _leg_camber.addTask(task);

}

void mgnss::controllers::WheelsControllerExtend::_setInitialConditions(){

        _steering_ptr->init();

        _dt = _robot.rate();

        _leg_steer.updateError();
        _leg_camber.updateError();
        _leg_castor.updateError();
        _steering_ptr->updateState();

        for(int i = 0; i < _steer_task.size(); i++)
                steerings[i] = _steer_task[i].getCurrent();

        _support.noalias() = _steering_ptr->getReference();
        _support_vel.setZero();


        for(int i = 0; i < _steer_task.size(); i++)
                _steer_task[i].setReference(steerings[i]);

        for(int i = 0; i < _camber_task.size(); i++)
                _camber_task[i].setReference(_camber_task[i].getCurrent());

        for(int i = 0; i < _caster_task.size(); i++)
                _caster_task[i].setReference(_caster_task[i].getCurrent());

}
