#include "mgnss/controllers/wheels_second_order.h"
// #include "mgnss/higher_level/steering_v8.h"
// #include "mgnss/higher_level/steering_reactif.h"
#include <mgnss/controllers/devel/contact_point_zmp_v2.h>

#include <mwoibn/hierarchical_control/tasks/contact_point_3D_rbdl_task.h>
#include <mwoibn/hierarchical_control/tasks/contact_point_second_order.h>

#include <mwoibn/robot_points/torus_model.h>
#include <mgnss/higher_level/previous_task.h>

#include <mwoibn/hierarchical_control/controllers/default.h>
#include <mwoibn/robot_points/handler.h>
#include <range/v3/range_for.hpp>

void mgnss::controllers::WheelsSecondOrder::compute()
{



        _com_ptr->update();
        std::cout << __PRETTY_FUNCTION__ << std::endl;
        //_support = _support + _support_vel*_robot.rate();
        // without resttering
        _command.noalias() = _ik_ptr->update();

        std::cout << _command.transpose() << std::endl;
        _robot.command.velocity.set(_command, _select_ik);


        _command.noalias() = _command * _robot.rate();
        _command.noalias() +=
                _robot.state.position.get();

        _robot.command.position.set(_command, _select_ik);
}

double mgnss::controllers::WheelsSecondOrder::limit(const double th)
{
        return th - 6.28318531 * std::floor((th + 3.14159265) / 6.28318531);
}

void mgnss::controllers::WheelsSecondOrder::_create(YAML::Node config){
        _createTasks(config);
        _initIK(config);
        _allocate();
}

// void mgnss::controllers::WheelsSecondOrder::resetSteering()
// {
//         for (auto& task_: _leg_tasks["STEERING"].second)
//                 task_.setReference(0);
// }

// void mgnss::controllers::WheelsSecondOrder::steering()
// {
//
//
//         // _steering_ref_ptr->compute(_next_step);
//
//         // steerings.noalias() = _steering_ref_ptr->get();
//
//         // for (int i = 0; i < 4; i++)
//         // {
//                  // steerings[i] = (steerings[i] < _l_limits[i]) ? steerings[i] + mwoibn::PI : steerings[i];
//                  // steerings[i] = (steerings[i] > _u_limits[i]) ? steerings[i] - mwoibn::PI : steerings[i];
//                  // setSteering(i, steerings[i]);
//         // }
// }


void mgnss::controllers::WheelsSecondOrder::_setInitialConditions(){

        _steering_ptr->reset();

        _dt = _robot.rate();

        // _leg_tasks["STEERING"].first.updateError();
        _leg_tasks["CAMBER"].first.updateError();
        _leg_tasks["CASTER"].first.updateError();

        _steering_ptr->updateState();

        _support.noalias() = _steering_ptr->getReference();
        _modified_support.setZero(_support.size());
        _support_vel.setZero();

        for(auto& item_: _leg_tasks)
            for(auto& angle_: item_.second.second)
                  angle_.reset();

                  _pelvis_orientation_ptr->points().point(0).getOrientationWorld(_orientation);
                  _heading = _orientation.swingTwist(_robot.contacts().contact(0).getGroundNormal(), _orientation).angle();
                  _pelvis_orientation_ptr->setReference(0, mwoibn::Quaternion::fromAxisAngle(_robot.contacts().contact(0).getGroundNormal(), _heading)*(_orientation));

        _pelvis_position_ptr->points().point(0).getLinearWorld(_position);
        // DIRECT COM CONTROL

        _pelvis_position_ptr->setReference(0, _position);
        _com_ptr->setReference(_position);

        _robot.centerOfMass().update(true);

        _tasks["CONSTRAINTS"]->update();
        _tasks["BASE"]->update();
        _tasks["CAMBER"]->update();
        // _tasks["STEERING"]->update();


        // shape_joint__->equality[0].jacobian = _tasks["CONSTRAINTS"]->getJacobian();
        // shape_joint__->equality[1].jacobian = _tasks["BASE"]->getJacobian();
        // shape_joint__->equality[2].jacobian = _tasks["CAMBER"]->getJacobian();

        // qr_tracking->update();

}

void mgnss::controllers::WheelsSecondOrder::step(){
        _position += _linear_vel  * _robot.rate();
        _heading  += _angular_vel[2] * _robot.rate();
        _heading  -= 6.28318531 * std::floor((_heading + 3.14159265) / 6.28318531); // limit -pi:pi
}

void mgnss::controllers::WheelsSecondOrder::_allocate(){

          _angular_vel.setZero();
          _linear_vel.setZero();

          // _select_steer = _robot.getDof(_robot.getLinks("camber"));
          // steerings.setZero(_select_steer.size());

          _support.setZero( _robot.getLinks("wheels").size()*3);
          _support_vel.setZero( _robot.getLinks("wheels").size()*3);

          _l_limits.setZero(_select_steer.size());
          _u_limits.setZero(_select_steer.size());

          _robot.lower_limits.position.get(_l_limits, _select_steer);
          _robot.upper_limits.position.get(_u_limits, _select_steer);

          _previous_command = mwoibn::VectorN::Zero(3);
          _command.setZero(_robot.getDofs());


  for(int i =0; i< _robot.getDofs(); i++)
     _log_names.push_back(_robot.getLinks(i));


        _com_ref.setZero(2);
        __last_steer.setZero(4);
        _zero.setZero( _robot.getLinks("wheels").size()*3);

        // qr_tracking->init();
        state_machine__->init();


}

void mgnss::controllers::WheelsSecondOrder::_initIK(YAML::Node config){


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
    for(auto entry : ik)   _ik_ptr->addAction(_createAction(entry.as<std::string>(), tunning));

    _ik_ptr->update();


    // YAML::Node steering = config["steerings"][config["steering"].as<std::string>()];

    // _steering_ref_ptr.reset(new mgnss::higher_level::SteeringReactif(
              // _robot, *_steering_ptr, _support_vel, steering["icm"].as<double>(), steering["sp"].as<double>(), steering["tracking"].as<double>(), _robot.rate(), steering["damp_icm"].as<double>(), steering["damp_sp"].as<double>(), steering["damp"].as<double>()));

}



void mgnss::controllers::WheelsSecondOrder::_createTasks(YAML::Node config){

        _name = config["name"].as<std::string>();


        _constraints_ptr.reset(
                new mwoibn::hierarchical_control::tasks::Constraints(_robot));
        _tasks["CONSTRAINTS"] = _constraints_ptr.get();

        _pelvis_orientation_ptr.reset(
                new mwoibn::hierarchical_control::tasks::OrientationSelective(
                        mwoibn::point_handling::OrientationsHandler("ROOT", _robot,
                                                                    _robot.getLinks("base")),
                        mwoibn::Vector3::Ones(), _robot));

        _tasks["BASE_ORIENTATION"] = _pelvis_orientation_ptr.get();
                mwoibn::Axis ax_;

                for(auto& name_: {"CASTER", "CAMBER"})
                        _leg_tasks[name_] = {mwoibn::hierarchical_control::tasks::Aggravated(), std::vector<mwoibn::hierarchical_control::tasks::Angle>{}};

                for(auto& name_: _robot.getLinks("wheels")){
                  ax_  = mwoibn::Axis(config["reference_axis"][name_]["x"].as<double>(),
                                     config["reference_axis"][name_]["y"].as<double>(),
                                     config["reference_axis"][name_]["z"].as<double>());
                  _leg_tasks["CAMBER"].second.push_back(mwoibn::hierarchical_control::tasks::Angle(
                             mwoibn::robot_class::angles::Camber(_robot,  mwoibn::point_handling::Frame(name_, _robot.getModel(), _robot.state), ax_), _robot));
                  // _leg_tasks["STEERING"].second.push_back(mwoibn::hierarchical_control::tasks::Angle(
                             // mwoibn::robot_class::angles::Steering(_robot, mwoibn::point_handling::Frame(name_, _robot.getModel(), _robot.state), ax_), _robot));
                }

                for(auto& name_: _robot.getLinks("camber")){
                  ax_  = mwoibn::Axis(config["reference_axis"][name_]["x"].as<double>(),
                                     config["reference_axis"][name_]["y"].as<double>(),
                                     config["reference_axis"][name_]["z"].as<double>());
                  _leg_tasks["CASTER"].second.push_back(mwoibn::hierarchical_control::tasks::Angle(
                             mwoibn::robot_class::angles::Caster(_robot,  mwoibn::point_handling::Frame(name_, _robot.getModel(), _robot.state), ax_), _robot));
                }

                for(auto& item_: _leg_tasks){
                    _tasks[item_.first] = &item_.second.first;
                    for(auto& angle_: item_.second.second)
                          item_.second.first.addTask(angle_);

                }
        mwoibn::Vector3 pelvis;
        // DIRECT COM
        // pelvis << 0, 0, 1;
        // INDIRECT CoM
        pelvis << 1, 1, 1;
        mwoibn::point_handling::PositionsHandler pelvis_ph("ROOT", _robot,
                                                           _robot.getLinks("base"));
        _pelvis_position_ptr.reset(
                new mwoibn::hierarchical_control::tasks::CartesianSelective(pelvis_ph,
                                                                            pelvis));

        // _steering_select.setConstant(4,false);

        _com_ptr.reset(new mwoibn::hierarchical_control::tasks::CenterOfMass(_robot));
        _com_ptr->setDofs(_robot.selectors().get("lower_body").getBool());

        std::vector<std::string> names = {"wheel_1", "wheel_2", "wheel_3", "wheel_4"};

        mwoibn::robot_points::Handler<mwoibn::robot_points::TorusModel> contact_points(_robot.getDofs());

          _contact_point.reset(new mwoibn::hierarchical_control::tasks::Aggravated());

            YAML::Node tunning = config["tunnings"][config["tunning"].as<std::string>()];

            std::cout << tunning << std::endl;

            _steering_ptr.reset( new mwoibn::hierarchical_control::tasks::ContactPointZMPV2(
                                _robot.getLinks("wheels"), _robot, config, _world, "ROOT", tunning["COP"].as<double>() ));

            state_machine__.reset(new mgnss::higher_level::StateMachine(_robot, config ));


            _steering_ptr->subscribe(true, true, false);
            _contact_point->addTask(*_steering_ptr);

          _tasks["CONTACT_POINTS"] = _contact_point.get();
          _tasks["CONTACT_POINTS_1"] = _steering_ptr.get();



        _world_posture_ptr.reset(new mwoibn::hierarchical_control::tasks::Aggravated());

        _world_posture_ptr->addTask(*_pelvis_orientation_ptr);

        // INDIRECT COM TRACKING
        mwoibn::VectorBool select(3);
        select << true, true, true;
        _world_posture_ptr->addTask(*_pelvis_position_ptr, select);
        _tasks["BASE_GRAVITY"] = _pelvis_position_ptr.get();
        _tasks["BASE"] = _world_posture_ptr.get();

}


std::shared_ptr<mwoibn::hierarchical_control::actions::Task> mgnss::controllers::WheelsSecondOrder::_taskAction(std::string task, YAML::Node config, std::string type){

    double ratio = config["ratio"].as<double>(); // 4
    double damp_ = config["damping"].as<double>();
    double task_damp_;

    if(!_tasks[task])
        throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": Unknown task '") + task + std::string("'."));
    if(!config[task])
            throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": Please defined gain for task '") + task + std::string("'."));

    task_damp_ = config[task+"_DAMP"] ? config[task+"_DAMP"].as<double>() : damp_;

    std::cout << "\t" << task << ": " << config[task] << "\t" << task_damp_ << std::endl;

    if(type == "" || type == "ns"){

    if(config[task].IsScalar())
        return std::make_shared<mwoibn::hierarchical_control::actions::Compute>(*_tasks[task], config[task].as<double>(),  task_damp_, _ik_ptr->state);

    else if (!config[task].IsSequence())
      throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": Unknown gain type for  '") + task + std::string("'."));

    if(config[task].size() != _tasks[task]->getTaskSize())
      throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": Incompatible gain size for task '") + task + std::string("' of size ") + std::to_string(_tasks[task]->getTaskSize()) + std::string("."));

      mwoibn::VectorN gain(config[task].size());

      for(int i = 0 ; i < config[task].size(); i++)
        gain[i] = config[task][i].as<double>();

      return std::make_shared<mwoibn::hierarchical_control::actions::Compute>(*_tasks[task], gain,  task_damp_, _ik_ptr->state);
    }

    if(type != "qp")
      throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": Unknown task type '") + type  + std::string("'."));

    _qr_wrappers.push_back(std::unique_ptr<mgnss::higher_level::QrTaskWrapper>(new mgnss::higher_level::QrTaskWrapper(*_tasks[task], config[task].as<double>(), task_damp_, _robot)));

    RANGES_FOR(auto& action, ranges::view::slice(_actions, 0, ranges::end-1) )
      _qr_wrappers.back()->equality.add(mgnss::higher_level::PreviousTask(*_tasks[action.first], _ik_ptr->state.command));
    //ptr->equality.add(mgnss::higher_level::PreviousTask(*_tasks["STEERING"], _ik_ptr->state.command));
    // qr_tracking->equality.add(mgnss::higher_level::PreviousTask(*_tasks["BASE"], _ik_ptr->state.command));
    _qr_wrappers.back()->init();
    return std::make_shared<mwoibn::hierarchical_control::actions::QP>(*_qr_wrappers.back(), _ik_ptr->state);
}

mwoibn::hierarchical_control::actions::Task& mgnss::controllers::WheelsSecondOrder::_createAction(std::string task, YAML::Node config){

  std::string name;
  std::string type = mwoibn::std_utils::separate(task, "::", name);


  if(!_actions[name])
      _actions[name] = _taskAction(name, config, type);

    return *_actions[name];
}


void mgnss::controllers::WheelsSecondOrder::nextStep()
{


      _robot.centerOfMass().update();

      _support += _support_vel*_robot.rate();

      // _tasks["BASE"]->update();
      // _tasks["CONSTRAINTS"]->update();
      // _tasks["CAMBER"]->update();
      // _tasks["STEERING"]->update();
      // mwoibn::VectorN _zero = mwoibn::VectorN::Zero(8);
      // state_machine__->update(_zero);

  //    qr_tracking->equality[0].jacobian = _tasks["CONSTRAINTS"]->getJacobian();
  //    qr_tracking->equality[1].jacobian = _tasks["BASE"]->getJacobian(); // it is not updated yet?
  //    qr_tracking->equality[2].jacobian = _tasks["CAMBER"]->getJacobian();
//
        // qr_tracking->solve();

        // std::cout << "qr_tracking->get()\t" << qr_tracking->get().transpose() << std::endl;

      step();

      updateBase();
      _updateSupport();

      _next_step[0] = (_linear_vel[0]);
      _next_step[1] = (_linear_vel[1]);
      _next_step[2] = (_angular_vel[2]); // just limit the difference

      // steering();

}


void mgnss::controllers::WheelsSecondOrder::fullUpdate(const mwoibn::VectorN& support)
{
        _robot.get();
        _robot.updateKinematics();

        setSupport(support);
        update();

        _robot.send();
        _robot.wait();
}


void mgnss::controllers::WheelsSecondOrder::log(mwoibn::common::Logger& logger, double time){
  logger.add("time", time);

   logger.add("th", _robot.state.position.get()[5]);
   logger.add("r_th", _heading);
   //
       for(int i = 0; i < 3; i++){

         logger.add(std::string("com_") + char('x'+i), _robot.centerOfMass().get()[i]);
         logger.add(std::string("r_base_") + char('x'+i), getBaseReference()[i]);
         logger.add(std::string("base_") + char('x'+i), _steering_ptr->base.get()[i]);

         for(int k = 0; k < 4; k++){

           logger.add("cp_"   + std::to_string(k+1) + "_" + char('x'+i), _steering_ptr->getPointStateReference(k)[i]);

           logger.add("r_cp_" + std::to_string(k+1) + "_" + char('x'+i), _steering_ptr->getReference()[k*3+i]);

         }
       }

        // qr_tracking->log(logger);
//
}
