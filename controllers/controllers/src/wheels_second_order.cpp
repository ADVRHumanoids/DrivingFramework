#include "mgnss/controllers/wheels_second_order.h"
// #include "mgnss/higher_level/steering_v8.h"
// #include "mgnss/higher_level/steering_reactif.h"
#include <mgnss/controllers/devel/contact_point_zmp_v2.h>

#include <mwoibn/hierarchical_control/tasks/contact_point_3D_rbdl_task.h>
#include <mwoibn/hierarchical_control/tasks/contact_point_second_order.h>

#include <mwoibn/robot_points/torus_model.h>
#include <mgnss/higher_level/previous_task.h>
#include <mgnss/higher_level/qp/constraints/joint_constraint.h>

#include <mwoibn/hierarchical_control/controllers/default.h>
#include <mwoibn/robot_points/handler.h>
#include <mwoibn/point_handling/spatial_velocity.h>
#include <range/v3/range_for.hpp>

void mgnss::controllers::WheelsSecondOrder::compute()
{



        _com_ptr->update();
        std::cout << __PRETTY_FUNCTION__ << std::endl;
        //_support = _support + _support_vel*_robot.rate();
        // without resttering
        state_machine__->update();

        _command.noalias() = _ik_ptr->update();

        for(int i = 0; i < 4; i++){
          double weight = std::fabs( _leg_tasks["CAMBER"].first.getTask(i).getJacobian()(0, 6*(i+1)+3) / _leg_tasks["CAMBER"].first.getTask(i).getJacobian()(0, 6*(i+1)+4)  );
          // double weight_2 = std::fabs( _tasks["CONTACT_POINTS_1"]->getJacobian()(2*i+1, 6*(i+1)+4) / _tasks["CONTACT_POINTS_1"]->getJacobian()(2*i+1, 6*(i+1)+0)  );

          // _qr_wrappers.back()->damping()[6*(i+1)] = 0.000002+std::fabs(_tasks["CONTACT_POINTS_1"]->getJacobian()(2*i+1, 6*(i+1)+4))*std::tanh(0.1*weight_2);

          _leg_tasks["CAMBER"].first.setWeight(3.0, i);
          _leg_tasks["CASTER"].first.setWeight(0.05*(1-std::tanh( 0.2*std::pow(weight,3) ) ), i);

          // std::cout << "weight\t" << weight_2 << std::endl;
          std::cout << "weight\t" << _leg_tasks["CAMBER"].first.getWeight(i);
          // std::cout << "\t" << _leg_tasks["CASTER"].first.getWeight(i) << std::endl;
        }
        // std::cout << "damping\t" << _qr_wrappers.back()->damping().transpose() << std::endl;

        // std::cout << _command.transpose() << std::endl;
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
        _allocate();
        _initIK(config);
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

        state_machine__->update();
        // restore__->update();
        _qr_wrappers["SHAPE"]->update();

        _tasks["CONSTRAINTS"]->update();
        _tasks["BASE"]->update();
        _tasks["CAMBER"]->update();

        _qr_wrappers["SHAPE_WHEEL"]->update();
        _qr_wrappers["SHAPE_JOINT"]->update();


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


        // _qr_wrappers["SHAPE_JOINT"]->init();
        // _qr_wrappers["SHAPE"]->init();

        // _qr_wrappers["SHAPE_JOINT"]->equality.add(mgnss::higher_level::PreviousTask(*_tasks["CONSTRAINTS"], _ik_ptr->state.command));
        // _qr_wrappers["SHAPE_JOINT"]->equality.add(mgnss::higher_level::PreviousTask(*_tasks["BASE"], _ik_ptr->state.command));

        // _qr_wrappers["SHAPE_WHEEL"]->init();
        // _qr_wrappers["SHAPE_JOINT"]->init();


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
    for(auto entry : ik)   _ik_ptr->addAction(_createAction(entry.as<std::string>(), tunning, config));



    // YAML::Node steering = config["steerings"][config["steering"].as<std::string>()];

    // _steering_ref_ptr.reset(new mgnss::higher_level::SteeringReactif(
              // _robot, *_steering_ptr, _support_vel, steering["icm"].as<double>(), steering["sp"].as<double>(), steering["tracking"].as<double>(), _robot.rate(), steering["damp_icm"].as<double>(), steering["damp_sp"].as<double>(), steering["damp"].as<double>()));

    _ik_ptr->update();

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

            _pelvis.reset(new mwoibn::robot_points::LinearPoint("pelvis", _robot));
            _steering_ptr.reset( new mwoibn::hierarchical_control::tasks::ContactPointSecondOrder(
                                _robot.getLinks("wheels"), _robot, config, *_pelvis, "pelvis" ));

            // _steering_ptr.reset( new mwoibn::hierarchical_control::tasks::ContactPointSecondOrder(
            //                       _robot.getLinks("wheels"), _robot, config, _world, "ROOT" ));

            state_machine__.reset(new mgnss::higher_level::StateMachine(_robot, config ));

            _qr_wrappers["SHAPE"] = std::unique_ptr<mgnss::higher_level::SupportShapingV4>(new mgnss::higher_level::SupportShapingV4(_robot, config, state_machine__->steeringFrames(), state_machine__->margin(), state_machine__->workspace()));
            _qr_wrappers["SHAPE_WHEEL"] = std::unique_ptr<mgnss::higher_level::QRJointSpaceV2>(new mgnss::higher_level::QRJointSpaceV2(*_qr_wrappers["SHAPE"], state_machine__->stateJacobian(), state_machine__->stateOffset(), _robot ));
            _qr_wrappers["SHAPE_JOINT"] = std::unique_ptr<mgnss::higher_level::QRJointSpaceV2>(new mgnss::higher_level::QRJointSpaceV2(*_qr_wrappers["SHAPE_WHEEL"], state_machine__->wheelOrientation().jacobian(), _zero, _robot ));
                        // shape_wheel__.reset(new mgnss::higher_level::QRJointSpaceV2(*shape__, state_machine__->stateJacobian(), state_machine__->stateOffset(), _robot ));
            // shape_joint__.reset(new mgnss::higher_level::QRJointSpaceV2(*shape_wheel__, state_machine__->wheelOrientation().jacobian(), _zero, _robot ));


            _steering_ptr->subscribe(true, true, false);
            _contact_point->addTask(*_steering_ptr);
            // _contact_point->addTask(*_tasks["CAMBER"]);
            // _contact_point->addTask(*_tasks["CASTER"]);

          _tasks["CONTACT_POINTS"] = _contact_point.get();
          _tasks["CONTACT_POINTS_1"] = _steering_ptr.get();
          // _tasks["CONTACT_POINTS_2"] = _tasks["CAMBER"];
          // _tasks["CONTACT_POINTS_3"] = _tasks["CASTER"];



        _world_posture_ptr.reset(new mwoibn::hierarchical_control::tasks::Aggravated());

        _world_posture_ptr->addTask(*_pelvis_orientation_ptr);

        // INDIRECT COM TRACKING
        mwoibn::VectorBool select(3);
        select << true, true, true;
        _world_posture_ptr->addTask(*_pelvis_position_ptr, select);
        _tasks["BASE_GRAVITY"] = _pelvis_position_ptr.get();
        _tasks["BASE"] = _world_posture_ptr.get();


        state_machine__->init();
        state_machine__->update();

}


std::shared_ptr<mwoibn::hierarchical_control::actions::Task> mgnss::controllers::WheelsSecondOrder::_taskAction(std::string task, YAML::Node config, std::string type, YAML::Node full_config){

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

      for(auto& action: _actions){
          if(action.first == task) continue;
          _qp_aggravated.back()->equality.add(mgnss::higher_level::PreviousTask(*_tasks[action.first], _ik_ptr->state.command));
      }
      _qp_aggravated.back()->hard_inequality.add(mgnss::higher_level::JointConstraint(_robot, mwoibn::eigen_utils::iota(_robot.getDofs()), {"POSITION","VELOCITY"}));

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

    for(auto& action: _actions){
        if(action.first == task) continue;
        if(!_tasks[action.first]) continue;
        _qr_wrappers[task]->equality.add(mgnss::higher_level::PreviousTask(*_tasks[action.first], _ik_ptr->state.command));
    }

    _qr_wrappers[task]->hard_inequality.add(mgnss::higher_level::JointConstraint(_robot, mwoibn::eigen_utils::iota(_robot.getDofs()), {"POSITION","VELOCITY"}));

    _qr_wrappers[task]->init();
    return std::make_shared<mwoibn::hierarchical_control::actions::QP>(*_qr_wrappers[task], _ik_ptr->state);



}

double mgnss::controllers::WheelsSecondOrder::_readTask(YAML::Node config, std::string task, mwoibn::VectorN& gain){

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

mwoibn::hierarchical_control::actions::Task& mgnss::controllers::WheelsSecondOrder::_createAction(std::string task, YAML::Node config, YAML::Node full_config){

  std::string name;
  std::string type = mwoibn::std_utils::separate(task, "::", name);


  if(!_actions[name])
      _actions[name] = _taskAction(name, config, type, full_config);

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

       for(int i = 0; i < 4; i++){
         logger.add("camber_"   + std::to_string(i) , _tasks["CAMBER"]->getJacobian()(i,6*(i+1)+3)/_tasks["CAMBER"]->getJacobian()(i,6*(i+1)+4));
         logger.add("point_"   + std::to_string(i) , _tasks["CONTACT_POINTS"]->getJacobian()(2*i+1,6*(i+1)+3)/_tasks["CONTACT_POINTS"]->getJacobian()(2*i+1,6*(i+1)+4));
         mwoibn::point_handling::FramePlus frame_temp__(_robot.getLinks("wheels")[i], _robot.getModel(), _robot.state);
         mwoibn::point_handling::SpatialVelocity temp__(frame_temp__);
         logger.add("wheels_x" + std::to_string(i), temp__.angular().getWorld()[0]);
         logger.add("wheels_y" + std::to_string(i), temp__.angular().getWorld()[1]);
         logger.add("wheels_z" + std::to_string(i), temp__.angular().getWorld()[2]);
       }

       std::cout << "final camber\t" << (_tasks["CAMBER"]->getJacobian()*_robot.command.velocity.get()).transpose() << std::endl;
        // qr_tracking->log(logger);
//
}
