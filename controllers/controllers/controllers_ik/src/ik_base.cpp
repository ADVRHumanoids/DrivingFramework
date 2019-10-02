#include "mgnss/controllers/ik_base.h"
#include <mwoibn/hierarchical_control/controllers/default.h>
#include <mgnss/higher_level/previous_task.h>
#include <mgnss/higher_level/qp/constraints/joint_constraint.h>


mgnss::controllers::IKBase::IKBase(mwoibn::robot_class::Robot& robot)
        : mgnss::modules::Base(robot)
{

        _ik_ptr.reset(new mwoibn::hierarchical_control::controllers::Actions(_robot.rate(), _robot.getDofs()));


        _x << 1, 0, 0;
        _y << 0, 1, 0;
        _z << 0, 0, 1;
}

void mgnss::controllers::IKBase::compute()
{
        _command.noalias() = _ik_ptr->update();

        for(int i = 0; i < _select_ik.size(); i++)
          _active_state[i] = _command[_select_ik[i]];

        _robot.command.velocity.set(_active_state, _select_ik);
        _command.noalias() = _command * _robot.rate();

        _robot.command.position.get(_active_state, _select_ik);

        for(int i = 0; i < _select_ik.size(); i++)
          _active_state[i] += _command[_select_ik[i]];

        _robot.command.position.set(_active_state, _select_ik);
}

void mgnss::controllers::IKBase::_create(YAML::Node config){
        _createTasks(config);
        _allocate();
        _initIK(config);
}


// void mgnss::controllers::IKBase::_setInitialConditions()
// {
//       _pelvis_orientation_ptr->points().point(0).getOrientationWorld(_orientation);
//       _heading = _orientation.swingTwist(_robot.contacts().contact(0).getGroundNormal(), _orientation).angle();
//       //_pelvis_orientation_ptr->setReference(0, mwoibn::Quaternion::fromAxisAngle(_robot.contacts().contact(0).getGroundNormal(), _heading));
//       _pelvis_orientation_ptr->setReference(0,mwoibn::Quaternion());
//
// }

// void mgnss::controllers::IKBase::_createTasks(YAML::Node config){
//         if(!config["track"])
//               throw std::invalid_argument(std::string("Wheels Controller: configuration doesn't containt required filed 'track'."));
//
//         std::string group_ = config["track"].as<std::string>();
//         std::vector<std::string> names = _robot.getLinks(group_);
//
//         // add wheels to the contact group
//         for(auto& name: names){
//           auto contact = ranges::find_if(_robot.contacts(), [&](auto& contact)-> bool{return _robot.getBodyName(contact->wrench().getBodyId()) == name;});
//           if ( contact != ranges::end(_robot.contacts()) )
//             _robot.contacts().toGroup((*contact)->getName(), group_);
//         }
//
//         // Set-up hierachical controller
//         _constraints_ptr.reset(
//                 new mwoibn::hierarchical_control::tasks::Constraints(_robot, {group_}));
//         _tasks["CONSTRAINTS"] = _constraints_ptr.get();
//
//         _pelvis_orientation_ptr.reset(
//                 new mwoibn::hierarchical_control::tasks::OrientationSelective(
//                         mwoibn::point_handling::OrientationsHandler("ROOT", _robot,
//                                                                     _robot.getLinks("base")),
//                         mwoibn::Vector3::Ones(), _robot));
//
//         _tasks["BASE_ORIENTATION"] = _pelvis_orientation_ptr.get();
//
// }

std::shared_ptr<mwoibn::hierarchical_control::actions::Task> mgnss::controllers::IKBase::_taskAction(std::string task, YAML::Node config, std::string type, YAML::Node full_config){

    // double ratio = config["ratio"].as<double>(); // 4

    std::cout << "\t" << task << std::endl;

    if(type == "qA"){
      // Check if task has been defined in th config file
      if(!full_config[task])
          throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": Unknown task '") + task + std::string("'."));

      _qp_aggravated.push_back(std::unique_ptr<mgnss::higher_level::QpAggravated>(new mgnss::higher_level::QpAggravated(_robot.getDofs())));

      // Initialize default constraints
      int eq_count = 0;
      for(auto& action: _actions){
          if(action.first == task) continue;
          ++eq_count;
      }


      // Add basic tasks
      for(int i = 0; i < full_config[task].size(); i++){
        std::string task_name_ = full_config[task][i].as<std::string>();

        std::cout << "\t" << task_name_ << std::endl;

        if(!_qr_wrappers[task_name_])
        {
          _taskAction(task_name_, config, "qp", full_config);
          _qr_wrappers[task_name_]->hard_inequality.remove(-1);

          for(auto&& zip: ranges::view::ints(0,eq_count))
            _qr_wrappers[task_name_]->equality.remove(-1);
        }

        _qp_aggravated.back()->add(*_qr_wrappers[task_name_]);

        _qr_wrappers[task_name_]->init();
      }

      // for(auto& action: _actions){
      //     if(action.first == task) continue;
      //     _qp_aggravated.back()->equality.add(mgnss::higher_level::PreviousTask(*_tasks[action.first], _ik_ptr->state.command));
      // }
      // _qp_aggravated.back()->hard_inequality.add(mgnss::higher_level::JointConstraint(_robot, mwoibn::eigen_utils::iota(_robot.getDofs()), {"POSITION","VELOCITY"}));
      _addConstraints(config, *_qp_aggravated.back(), task); // this should add the constrinats before the ones that should be removed

      _qp_aggravated.back()->init();
      return std::make_shared<mwoibn::hierarchical_control::actions::QP>(*_qp_aggravated.back(), _ik_ptr->state);
    }

    if(type == "" || type == "ns"){
        mwoibn::VectorN gain;
        double task_damp_ = _readTask(config, task, gain);
        return std::make_shared<mwoibn::hierarchical_control::actions::Compute>(*_tasks[task], gain,  task_damp_, _ik_ptr->state);
    }

    if( type != "qp")
      throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": Unknown task type '") + type  + std::string("'."));


    if(!_qr_wrappers[task]){
        mwoibn::VectorN gain;
        double task_damp_ = _readTask(config, task, gain);
       _qr_wrappers[task] = std::unique_ptr<mgnss::higher_level::QrTaskWrapper>(new mgnss::higher_level::QrTaskWrapper(*_tasks[task], gain, task_damp_, _robot));
    }

    _addConstraints(config, *_qr_wrappers[task], task);
    _addConstraints(config, *_qr_wrappers[task]); // this should add the constrinats before the ones that should be removed

    _qr_wrappers[task]->init();
    return std::make_shared<mwoibn::hierarchical_control::actions::QP>(*_qr_wrappers[task], _ik_ptr->state);



}




double mgnss::controllers::IKBase::_readTask(YAML::Node config, std::string task, mwoibn::VectorN& gain){

  double damp_ = config["damping"].as<double>();
  // double task_damp_;

  if(!_tasks[task])
      throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": Unknown task '") + task + std::string("'."));
  if(!config[task])
          throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": Please defined gain for task '") + task + std::string("'."));

  double task_damp_ = config[task+"_DAMP"] ? config[task+"_DAMP"].as<double>() : damp_;

  std::cout << "\t" << task << ": " << config[task] << "\t" << task_damp_ << std::endl;


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

  return task_damp_;

}


mwoibn::hierarchical_control::actions::Task& mgnss::controllers::IKBase::_createAction(std::string task, YAML::Node config, YAML::Node full_config){

  std::string name;
  std::string type = mwoibn::std_utils::separate(task, "::", name);


  if(!_actions[name])
      _actions[name] = _taskAction(name, config, type, full_config);

    return *_actions[name];
}


void mgnss::controllers::IKBase::_initIK(YAML::Node config){

  std::cout << "Inverse Kinematics loaded " << config["tunning"] << " tunning." << std::endl;

  if(!config["chain"])
        throw std::invalid_argument(std::string("Inverse Kinematics: configuration doesn't containt required filed 'chain'."));
  _select_ik = _robot.getDof(_robot.getLinks(config["chain"].as<std::string>()));


  // {
  //     std::string name = _robot.getBodyName(contact->wrench().getBodyId());
  //
  //     if(!std::count(names.begin(), names.end(), name)){
  //       std::cout << "Tracked point " << name << " could not be initialized" << std::endl;
  //       names.erase(std::remove(names.begin(), names.end(), name), names.end());
  //       continue;
  // }

  _active_state.setZero(_select_ik.size());

  YAML::Node ik =  config["IK"];

  YAML::Node tunning = config["tunnings"][config["tunning"].as<std::string>()];

  std::cout << "Loaded tunning" << std::endl;
  for(auto entry : tunning)
        std::cout << "\t" << entry.first << ": " << entry.second << std::endl;

  std::cout << "Initializing Inverse Kinematics..." << std::endl;
  for(auto entry : ik)  _ik_ptr->addAction(_createAction(entry.as<std::string>(), tunning, config));

   _ik_ptr->update();

}

void mgnss::controllers::IKBase::_addConstraints(YAML::Node config, mgnss::higher_level::QrTask& task, const std::string& name){

  for(auto& action: _actions){
      if(action.first == name) continue;
      if(!_tasks[action.first]) continue;
      task.equality.add(mgnss::higher_level::PreviousTask(*_tasks[action.first], _ik_ptr->state.command));
  }

  task.hard_inequality.add(mgnss::higher_level::JointConstraint(_robot, mwoibn::eigen_utils::iota(_robot.getDofs()), {"POSITION","VELOCITY"}));
}


//
// void mgnss::controllers::IKBase::nextStep()
// {
//         _robot.centerOfMass().update();
//
//         step();
//
//         updateBase();
//         _updateSupport();
//
//         _next_step[0] =
//                 (_linear_vel[0]);
//         _next_step[1] =
//                 (_linear_vel[1]);
//         _next_step[2] =
//                 (_angular_vel[2]); // just limit the difference
//
//         steering();
// }

void mgnss::controllers::IKBase::_allocate(){
        // _previous_command = mwoibn::VectorN::Zero(3);
        _command.setZero(_robot.getDofs());
        // _active_state.setZero(_select_ik.size());
}
