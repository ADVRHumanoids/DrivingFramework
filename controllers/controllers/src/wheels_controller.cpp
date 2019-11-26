#include "mgnss/controllers/wheels_controller.h"
#include "mgnss/higher_level/steering_v4.h"
#include <mgnss/higher_level/previous_task.h>
#include <mgnss/higher_level/qp/constraints/joint_constraint.h>


mgnss::controllers::WheelsController::WheelsController(mwoibn::robot_class::Robot& robot)
        : IKBase(robot)
{}


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

void mgnss::controllers::WheelsController::compute()
{
        // std::cout << "compute\t" << _robot.command.position.get().transpose() << std::endl;

        _command.noalias() = _ik_ptr->update();

        // std::cout << "IK\t" << _command.transpose() << std::endl;
        // std::cout << "position\t" << _robot.command.velocity.set(_command, _select_ik) << std::endl;
        _robot.command.velocity.set(_command, _select_ik);
        _command.noalias() = _command * _robot.rate();
        //_active_state = _command;
        // std::cout << "position\t" << _robot.command.position.get().head<30>().transpose() << std::endl;

        _robot.command.position.get(_active_state, _select_ik);

        for(int i = 0; i < _select_ik.size(); i++)
        _command[i] += _active_state[i];

        _robot.command.position.set(_command, _select_ik);
        // std::cout << "after position\t" << _robot.command.position.get().head<30>().transpose() << std::endl;


}


void mgnss::controllers::WheelsController::_setInitialConditions()
{
      _pelvis_orientation_ptr->points().point(0).getOrientationWorld(_orientation);
      _heading = _orientation.swingTwist(_robot.contacts().contact(0).getGroundNormal(), _orientation).angle();
      //_pelvis_orientation_ptr->setReference(0, mwoibn::Quaternion::fromAxisAngle(_robot.contacts().contact(0).getGroundNormal(), _heading));
      _pelvis_orientation_ptr->setReference(0,mwoibn::Quaternion());

}

void mgnss::controllers::WheelsController::_createTasks(YAML::Node config){
        if(!config["track"])
              throw std::invalid_argument(std::string("Wheels Controller: configuration doesn't containt required filed 'track'."));

        std::string group_ = config["track"].as<std::string>();
        std::vector<std::string> names = _robot.getLinks(group_);

        // add wheels to the contact group
        for(auto& name: names){
          auto contact = ranges::find_if(_robot.contacts(), [&](auto& contact)-> bool{return _robot.getBodyName(contact->wrench().getBodyId()) == name;});
          if ( contact != ranges::end(_robot.contacts()) )
            _robot.contacts().toGroup((*contact)->getName(), group_);
        }

        // Set-up hierachical controller
        _constraints_ptr.reset(
                new mwoibn::hierarchical_control::tasks::Constraints(_robot, {group_}));
        _tasks["CONSTRAINTS"] = _constraints_ptr.get();

        _pelvis_orientation_ptr.reset(
                new mwoibn::hierarchical_control::tasks::OrientationSelective(
                        mwoibn::point_handling::OrientationsHandler("ROOT", _robot,
                                                                    _robot.getLinks("base")),
                        mwoibn::Vector3::Ones(), _robot));

        _tasks["BASE_ORIENTATION"] = _pelvis_orientation_ptr.get();

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

        //steering();
}

void mgnss::controllers::WheelsController::_allocate(){

        IKBase::_allocate();
        _angular_vel.setZero();
        _linear_vel.setZero();

        _select_steer = _robot.getDof(_robot.getLinks("camber"));
        steerings.setZero(_select_steer.size());

        _support.setZero( _robot.getLinks("wheels").size()*3);
        _support_vel.setZero( _robot.getLinks("wheels").size()*3);

        _l_limits.setZero(_select_steer.size());
        _u_limits.setZero(_select_steer.size());

        _robot.lower_limits.position.get(_l_limits, _select_steer);
        _robot.upper_limits.position.get(_u_limits, _select_steer);

        _previous_command = mwoibn::VectorN::Zero(3);
        //_command.setZero(_robot.getDofs());
}
