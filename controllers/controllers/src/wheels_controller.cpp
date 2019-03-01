#include "mgnss/controllers/wheels_controller.h"
#include "mgnss/higher_level/steering_v4.h"
#include <mwoibn/hierarchical_control/controllers/default.h>
#include <mgnss/higher_level/previous_task.h>
#include <mgnss/higher_level/joint_constraint.h>


mgnss::controllers::WheelsController::WheelsController(mwoibn::robot_class::Robot& robot)
        : mgnss::modules::Base(robot)
{

//  _ik_ptr.reset(new mwoibn::hierarchical_control::HierarchicalController());
//        _ik_ptr.reset(new mwoibn::hierarchical_control::controllers::Default());
        _ik_ptr.reset(new mwoibn::hierarchical_control::controllers::Actions(_robot.rate(), _robot.getDofs()));
//        _ik_ptr = dynamic_cast<mwoibn::hierarchical_control::controllers::Actions*>(_ik_ptr.get());


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
        _command.noalias() = _ik_ptr->update();

        _robot.command.velocity.set(_command, _select_ik);

        _command.noalias() = _command * _robot.rate();
        _command.noalias() +=
                _robot.state.position.get();

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

void mgnss::controllers::WheelsController::_setInitialConditions()
{
      _pelvis_orientation_ptr->points().point(0).getOrientationWorld(_orientation);
      _heading = _orientation.swingTwist(_robot.contacts().contact(0).getGroundNormal(), _orientation).angle();
      _pelvis_orientation_ptr->setReference(0, mwoibn::Quaternion::fromAxisAngle(_robot.contacts().contact(0).getGroundNormal(), _heading)*(_orientation));

}

void mgnss::controllers::WheelsController::_createTasks(YAML::Node config){

        // Set-up hierachical controller
        _constraints_ptr.reset(
                new mwoibn::hierarchical_control::tasks::Constraints(_robot));
        _tasks["CONSTRAINTS"] = _constraints_ptr.get();

        _pelvis_orientation_ptr.reset(
                new mwoibn::hierarchical_control::tasks::OrientationSelective(
                        mwoibn::point_handling::OrientationsHandler("ROOT", _robot,
                                                                    _robot.getLinks("base")),
                        mwoibn::Vector3::Ones(), _robot));

        _tasks["BASE_ORIENTATION"] = _pelvis_orientation_ptr.get();

}

std::shared_ptr<mwoibn::hierarchical_control::actions::Task> mgnss::controllers::WheelsController::_taskAction(std::string task, YAML::Node config, std::string type){


    double ratio = config["ratio"].as<double>(); // 4
    double damp_ = config["damping"].as<double>();
    double task_damp_;

    if(!_tasks[task])
        throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": Unknown task '") + task + std::string("'."));
    if(!config[task])
            throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": Please defined gain for task '") + task + std::string("'."));

    task_damp_ = config[task+"_DAMP"] ? config[task+"_DAMP"].as<double>() : damp_;

    std::cout << "\t" << task << ": " << config[task] << "\t" << task_damp_ << std::endl;

    mwoibn::VectorN gain;
    if(config[task].IsScalar())
          gain.setConstant(_tasks[task]->getTaskSize(), config[task].as<double>() );
    else if (!config[task].IsSequence())
            throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": Unknown gain type for  '") + task + std::string("'."));
    else{
      if(config[task].size() != _tasks[task]->getTaskSize())
          throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": Incompatible gain size for task '") + task + std::string("' of size ") + std::to_string(_tasks[task]->getTaskSize()) + std::string("."));

          gain.setZero(config[task].size());

          for(int i = 0 ; i < config[task].size(); i++)
          gain[i] = config[task][i].as<double>();
        }
    if(type == "" || type == "ns")
      return std::make_shared<mwoibn::hierarchical_control::actions::Compute>(*_tasks[task], gain,  task_damp_, _ik_ptr->state);

    if(type != "qp")
        throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": Unknown task type '") + type  + std::string("'."));

    _qr_wrappers.push_back(std::unique_ptr<mgnss::higher_level::QrTaskWrapper>(new mgnss::higher_level::QrTaskWrapper(*_tasks[task], gain, task_damp_, _robot)));

    for(auto& action: _actions){
        if(action.first == task) continue;
        _qr_wrappers.back()->equality.add(mgnss::higher_level::PreviousTask(*_tasks[action.first], _ik_ptr->state.command));
    }

    _qr_wrappers.back()->hard_inequality.add(mgnss::higher_level::JointConstraint(_robot, mwoibn::eigen_utils::iota(_robot.getDofs()), {"POSITION", "VELOCITY"}));

    _qr_wrappers.back()->init();
    return std::make_shared<mwoibn::hierarchical_control::actions::QP>(*_qr_wrappers.back(), _ik_ptr->state);

}

mwoibn::hierarchical_control::actions::Task& mgnss::controllers::WheelsController::_createAction(std::string task, YAML::Node config){

  std::string name;
  std::string type = mwoibn::std_utils::separate(task, "::", name);


  if(!_actions[name])
      _actions[name] = _taskAction(name, config, type);

    return *_actions[name];
}

void mgnss::controllers::WheelsController::_initIK(YAML::Node config){

  std::cout << "Wheeled Motion loaded " << config["tunning"] << " tunning." << std::endl;

  if(!config["chain"])
        throw std::invalid_argument(std::string("Wheels Controller: configuration doesn't containt required filed 'chain'."));
  _select_ik = _robot.getDof(_robot.getLinks(config["chain"].as<std::string>()));

  YAML::Node ik =  config["IK"];

  YAML::Node tunning = config["tunnings"][config["tunning"].as<std::string>()];

  std::cout << "Loaded tunning" << std::endl;
  for(auto entry : tunning)
        std::cout << "\t" << entry.first << ": " << entry.second << std::endl;

  std::cout << "Initializing Inverse Kinematics..." << std::endl;
  for(auto entry : ik)  _ik_ptr->addAction(_createAction(entry.as<std::string>(), tunning));

   _ik_ptr->update();

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

        _support.setZero( _robot.getLinks("wheels").size()*3);
        _support_vel.setZero( _robot.getLinks("wheels").size()*3);

        _l_limits.setZero(_select_steer.size());
        _u_limits.setZero(_select_steer.size());

        _robot.lower_limits.position.get(_l_limits, _select_steer);
        _robot.upper_limits.position.get(_u_limits, _select_steer);

        _previous_command = mwoibn::VectorN::Zero(3);
        _command.setZero(_robot.getDofs());

}
