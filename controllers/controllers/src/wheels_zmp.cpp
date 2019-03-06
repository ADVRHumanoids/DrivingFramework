#include "mgnss/controllers/wheels_zmp.h"
// #include "mgnss/higher_level/steering_v8.h"
#include "mgnss/higher_level/steering_v8.h"
#include "mgnss/higher_level/steering_reactif.h"

#include <mgnss/controllers/devel/contact_point_zmp_v2.h>

#include <mwoibn/hierarchical_control/tasks/contact_point_3D_rbdl_task.h>

#include <mwoibn/robot_points/torus_model.h>

#include <mwoibn/hierarchical_control/controllers/default.h>
#include <mwoibn/robot_points/handler.h>

void mgnss::controllers::WheelsZMP::compute()
{
        _com_ptr->update();
        //_support = _support + _support_vel*_robot.rate();
        // without resttering
        WheelsController::compute();
}

void mgnss::controllers::WheelsZMP::steering()
{


        _steering_ref_ptr->compute(_next_step);

        steerings.noalias() = _steering_ref_ptr->get();
        // std::cout << "_steerings\t" << steerings.transpose() << std::endl;

        for (int i = 0; i < 4; i++)
        {
                 // steerings[i] = (steerings[i] < _l_limits[i]) ? steerings[i] + mwoibn::PI : steerings[i];
                 // steerings[i] = (steerings[i] > _u_limits[i]) ? steerings[i] - mwoibn::PI : steerings[i];
                 setSteering(i, steerings[i]);
        }
}


void mgnss::controllers::WheelsZMP::_setInitialConditions(){

  // if(!_steering_select.all())
      _steering_ptr->reset();

  // if(_steering_select.any())
  //       _steering_ptr_2->reset();

        _dt = _robot.rate();

        _leg_tasks["STEERING"].first.updateError();
        _leg_tasks["CAMBER"].first.updateError();
        _leg_tasks["CASTER"].first.updateError();

  // if(!_steering_select.all())
        _steering_ptr->updateState();
  // if(_steering_select.any())
  //       _steering_ptr_2->updateState();


        // for(int i = 0, k = 0; i < _steering_select.size(); i++){
        //   _support.segment<3>(3*i) = (_steering_select[i])? _steering_ptr_2->getReference(k): _steering_ptr->getReference(i-k);
        //   k += _steering_select[i];
        // }
        _support.noalias() = _steering_ptr->getReference();
        _modified_support.setZero(_support.size());
        _support_vel.setZero();
        // _steer.setZero(4);

        for(auto& item_: _leg_tasks)
            for(auto& angle_: item_.second.second)
                  angle_.reset();

        WheelsController::_setInitialConditions();

        _pelvis_position_ptr->points().point(0).getLinearWorld(_position);
        // DIRECT COM CONTROL
        //_position.head<2>() = _robot.centerOfMass().get().head<2>();
        _pelvis_position_ptr->setReference(0, _position);
        _com_ptr->setReference(_position);

        // if(_steering_select.any())
        //       _steering_ptr_2->start();
        _robot.centerOfMass().update(true);
        mwoibn::VectorN beta(4);
        for(int i = 0; i <  _leg_tasks["STEERING"].second.size(); i++)
          beta[i] = _leg_tasks["STEERING"].second[i].getCurrent();

        state_machine__->update();

        _qr_wrappers["SHAPE"]->update();

        // _tasks["CONSTRAINTS"]->update();
        // _tasks["BASE"]->update();
        // _tasks["CAMBER"]->update();

//        _qr_wrappers["SHAPE_WHEEL"]->update();
        // _qr_wrappers["SHAPE_JOINT"]->update();

}

void mgnss::controllers::WheelsZMP::step(){
        // _steering_ptr->setVelocity(_modified_support);

        _position += _linear_vel  * _robot.rate();
        _heading  += _angular_vel[2] * _robot.rate();
        _heading  -= 6.28318531 * std::floor((_heading + 3.14159265) / 6.28318531); // limit -pi:pi
}

void mgnss::controllers::WheelsZMP::_allocate(){
          WheelsControllerExtend::_allocate();
        _com_ref.setZero(2);
        __last_steer.setZero(4);
        _modified_support.setZero( _robot.getLinks("wheels").size()*3);
        _zero.setZero( _robot.getLinks("wheels").size()*3);
        _qr_wrappers["SHAPE"]->init();
        // __dynamics.subscribe({mwoibn::dynamic_models::DYNAMIC_MODEL::INERTIA_INVERSE, mwoibn::dynamic_models::DYNAMIC_MODEL::INERTIA_INVERSE, mwoibn::dynamic_models::DYNAMIC_MODEL::NON_LINEAR});

}

void mgnss::controllers::WheelsZMP::_initIK(YAML::Node config){

    WheelsController::_initIK(config);


    YAML::Node steering = config["steerings"][config["steering"].as<std::string>()];
    std::cout << "Loaded sttering" << std::endl;
    for(auto entry : steering)
          std::cout << "\t" << entry.first << ": " << entry.second << std::endl;

    _steering_ref_ptr.reset(new mgnss::higher_level::SteeringReactif(
              _robot, *_steering_ptr, _support_vel, steering["icm"].as<double>(), steering["sp"].as<double>(), steering["tracking"].as<double>(), _robot.rate(), steering["damp_icm"].as<double>(), steering["damp_sp"].as<double>(), steering["damp"].as<double>()));

    // _steering_ref_ptr.reset(new mgnss::higher_level::Steering8(
    //                       _robot, *_steering_ptr, _support_vel, steering["icm"].as<double>(), steering["sp"].as<double>(), steering["tracking"].as<double>(), _robot.rate(), steering["damp_icm"].as<double>(), steering["damp_sp"].as<double>(), steering["damp"].as<double>()));
}


void mgnss::controllers::WheelsZMP::_createTasks(YAML::Node config){

        _name = config["name"].as<std::string>();

        WheelsControllerExtend::_createTasks(config);

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

        // for(auto& contact: _robot.contacts())
        // {
        //     std::string name = _robot.getBodyName(contact->wrench().getBodyId());
        //     if(!std::count(names.begin(), names.end(), name)){
        //       std::cout << "Tracked point " << name << " could not be initialized" << std::endl;
        //       names.erase(std::remove(names.begin(), names.end(), name), names.end());
        //       continue;
        //     }
        //     _steering_select[contact_points.size()] = true;
        //
        //     mwoibn::robot_points::TorusModel torus_(
        //                        _robot, mwoibn::point_handling::FramePlus(name,
        //                        _robot.getModel(), _robot.state),
        //                        mwoibn::Axis(config["reference_axis"][name]["x"].as<double>(),
        //                                     config["reference_axis"][name]["y"].as<double>(),
        //                                     config["reference_axis"][name]["z"].as<double>()),
        //                                     config["minor_axis"].as<double>(), config["major_axis"].as<double>(),
        //                                     contact->getGroundNormal());
        //     contact_points.add(torus_);
        //
        // }

          _contact_point.reset(new mwoibn::hierarchical_control::tasks::Aggravated());

          //_base_ptr.reset(new mwoibn::robot_points::LinearPoint(_robot.getLinks("base")[0], _robot));

          // if(_steering_select.count() < 4){
          //   _steering_ptr.reset(
          //         new mwoibn::hierarchical_control::tasks::ContactPoint3DRbdl({}, _robot, config, _robot.centerOfMass(), _robot.getLinks("base")[0]));
          //   _steering_ptr->subscribe(true, true, false);
          //   _contact_point->addTask(*_steering_ptr);
          // }
          // if(_steering_select.count()){
            // _steering_ptr_2.reset(new mwoibn::hierarchical_control::tasks::ContactPointZMP(contact_points,
            //                     _robot, "pelvis", config["position_gain"].as<double>()));
            YAML::Node tunning = config["tunnings"][config["tunning"].as<std::string>()];

            std::cout << tunning << std::endl;

            _steering_ptr.reset( new mwoibn::hierarchical_control::tasks::ContactPointZMPV2(
                                _robot.getLinks("wheels"), _robot, config, _world, "ROOT", tunning["COP"].as<double>()));
            state_machine__.reset(new mgnss::higher_level::StateMachine(_robot, config ));

            _qr_wrappers["SHAPE"] = std::unique_ptr<mgnss::higher_level::SupportShapingV4>(new mgnss::higher_level::SupportShapingV4(_robot, config, state_machine__->steeringFrames(), state_machine__->margin(), state_machine__->workspace()));
            // _qr_wrappers["SHAPE_WHEEL"] = std::unique_ptr<mgnss::higher_level::QRJointSpaceV2>(new mgnss::higher_level::QRJointSpaceV2(*_qr_wrappers["SHAPE"], state_machine__->stateJacobian(), state_machine__->stateOffset(), _robot ));
            // _qr_wrappers["SHAPE_JOINT"] = std::unique_ptr<mgnss::higher_level::QRJointSpaceV2>(new mgnss::higher_level::QRJointSpaceV2(*_qr_wrappers["SHAPE_WHEEL"], state_machine__->wheelOrientation().passJacobian(), _zero, _robot ));
            // _qr_wrappers["SHAPE_JOINT"] = std::unique_ptr<mgnss::higher_level::QRJointSpaceV2>(new mgnss::higher_level::QRJointSpaceV2(*_qr_wrappers["SHAPE"], state_machine__->stateJacobian(), state_machine__->stateOffset(), _robot ));

            _steering_ptr->subscribe(true, true, false);
            _contact_point->addTask(*_steering_ptr);

          _tasks["CONTACT_POINTS"] = _contact_point.get();
          _tasks["CONTACT_POINTS_1"] = _steering_ptr.get();
          // _tasks["CONTACT_POINTS_2"] = _steering_ptr_2.get();

        _world_posture_ptr.reset(new mwoibn::hierarchical_control::tasks::Aggravated());

        _world_posture_ptr->addTask(*_pelvis_orientation_ptr);

        // INDIRECT COM TRACKING
        mwoibn::VectorBool select(3);
        select << true, true, true;
        _world_posture_ptr->addTask(*_pelvis_position_ptr, select);
        // /_tasks["BASE_GROUND"] = _com_ptr.get();
        _tasks["BASE_GRAVITY"] = _pelvis_position_ptr.get();
        _tasks["BASE"] = _world_posture_ptr.get();

        // DIRECT COM TRACKING
        // _world_posture_ptr->addTask(*_com_ptr);

        // mwoibn::VectorBool select(3);
        // select << false, false, true;
        // _world_posture_ptr->addTask(*_pelvis_position_ptr, select);
        //
        //
        // _tasks["BASE_GROUND"] = _com_ptr.get();
        // _tasks["BASE_GRAVITY"] = _pelvis_position_ptr.get();
        // _tasks["BASE"] = _world_posture_ptr.get();

        state_machine__->init();
        state_machine__->update();


}




void mgnss::controllers::WheelsZMP::log(mwoibn::common::Logger& logger, double time){
  // logger.add("com_x", getComFull()[0]);
  // logger.add("com_y", getComFull()[1]);

  // logger.add("r_com_x", refCom()[0]);
  // logger.add("r_com_y", refCom()[1]);
  //mgnss::controllers::WheelsControllerExtend::log(logger ,time);
  logger.add("time", time);

   logger.add("th", _robot.state.position.get()[5]);
   logger.add("r_th", _heading);
   //
       for(int i = 0; i < 3; i++){

         // logger.add(std::string("cop_") + char('x'+i), _robot.centerOfPressure().get()[i]);
         logger.add(std::string("com_") + char('x'+i), _robot.centerOfMass().get()[i]);
         logger.add(std::string("r_base_") + char('x'+i), getBaseReference()[i]);
         logger.add(std::string("base_") + char('x'+i), _steering_ptr->base.get()[i]);

         for(int k = 0; k < 4; k++){

           logger.add("cp_"   + std::to_string(k+1) + "_" + char('x'+i), _steering_ptr->getPointStateReference(k)[i]);

           logger.add("r_cp_" + std::to_string(k+1) + "_" + char('x'+i), _steering_ptr->getReference()[k*3+i]);

           // logger.add("position_error_" + std::to_string(point+1) + "_" + char('x'+i),
           //                 _steering_select[point] ? _steering_ptr_2->getPositionError()[k*3+i] : 0);

           // logger.add("full_error_" + std::to_string(k+1) + "_" + char('x'+i), _steering_ptr->getFullError()[k*3+i]);
           //
           // logger.add("getForce_" + std::to_string(k+1) + "_" + char('x'+i), _steering_ptr->getForce()[k*3+i]);

           // logger.add("com_error_" + std::to_string(point+1) + "_" + char('x'+i),
           //                _steering_select[point] ? _steering_ptr_2->getTestError()[k*3+i] : 0);

           // k += _steering_select[point];

         }
       }

//      for(auto& state_: {"MINUS_TORQUE","CONTACT_TORQUE","COM_TORQUE"}){
//        for(int i = 0; i < _robot.state[state_].size(); i++)
//          logger.add(state_ + std::string("_") + _robot.getLinks(i), _robot.state[state_][i]);
//      }
//      for(auto& state_: {"CONTACT_FORCE","COM_FORCE", "COM_CONTACT_FORCE"}){
//          for(int i = 0; i < _robot.state[state_].size(); i++)
//            logger.add(state_ + std::string("_") + std::to_string(i/3+1) + std::string("_") + char('x'+(i%3)), _robot.state[state_][i]);
//      }

// sprawdzić siłę, a potem spróbować drugą metodę
// jeśli zadziała to ruszyć z support polygon i ewentualnie jak bede miec czas to pomyslec z admitance na contact point


        // CHECK FORCE ESTIMATION
        // __dynamics.update();
        // _robot.centerOfMass().accelerationComponent();
        //
        //
        // mwoibn::Vector3 F_m__ = _robot.centerOfMass().mass()*_robot.centerOfMass().getJacobian()*__dynamics.getInertiaInverse()*(-_robot.state["OVERALL_FORCE"].get());
        // F_m__ +=  _robot.centerOfMass().mass()*_robot.centerOfMass().acceleration();
        //
        // mwoibn::Vector3 F_des__ = F_m__;

        // this does not use a CoM.mass because it is already included in the gain in this implementation
        //F_des__.head<2>() += _tasks["BASE"]->getError().segment<2>(3).cwiseProduct(std::dynamic_pointer_cast<mwoibn::hierarchical_control::actions::Compute>(_actions["BASE"])->gain().segment<2>(3));
        // F_des__ += (_robot.centerOfMass().getJacobian()*_robot.command.velocity.get())*_robot.centerOfMass().mass()/_robot.rate(); // \d_q_des*CoM.m/robot.rate
        //
        //
        // for(int i = 0; i < 3; i++){
        //   logger.add(std::string("comForce_") + char('x'+i), F_m__[i]);
        //   logger.add(std::string("comDes_") + char('x'+i), F_des__[i]);
        //   logger.add(std::string("comDiff_") + char('x'+i), F_des__[i] - F_m__[i]);
        // }
        //
        // logger.add(std::string("comFactor"), dynamic_cast<mwoibn::hierarchical_control::tasks::ContactPointZMPV2*>(_steering_ptr.get())->comFactor() );
        // logger.add(std::string("forceFactor"), dynamic_cast<mwoibn::hierarchical_control::tasks::ContactPointZMPV2*>(_steering_ptr.get())->forceFactor() );

        // i need a snap in solver
        // shape__->solve(logger);
        // shape__->log(logger);

        // mwoibn::Vector3 temp__ = mwoibn::Vector3::Zero();
        // mwoibn::VectorN desired__ = state_machine__->desiredJacobian()*shape_joint__->get();
        // mwoibn::VectorN world__ = state_machine__->worldJacobian()*shape_joint__->get();
        // mwoibn::VectorN workspace__ = state_machine__->workspace().jacobian*desired__;
        // std::cout << "workspace velocity" << (workspace__).transpose() << std::endl;
        // std::cout << "workspace limit";
        // for (int i =0; i < workspace__.size(); i++)
        //  std::cout << "\t" << state_machine__->workspace().limit[i]*state_machine__->workspace().limit[i] << std::endl;
        // std::cout << std::endl;
        //
        //
        // std::cout << "workspace state\t";
        // for (int i =0; i < workspace__.size(); i++)
        //  std::cout << "\t" << state_machine__->workspace().state[i] << std::endl;
        // std::cout << std::endl;
        //
        // std::cout << "workspace est\t";
        // for (int i =0; i < workspace__.size(); i++)
        //  std::cout << "\t" << state_machine__->workspace().state[i] +  workspace__[i]*_robot.rate() << std::endl;
        // std::cout << std::endl;

        // std::cout << "steering\n" << state_machine__->stateJacobian()*shape_joint__->get() << std::endl;
        // std::cout << "desired steering\n" << _steer << std::endl;

        // std::cout << "joint space solution\n" << state_machine__->stateJacobian()*shape_joint__->get() << std::endl;
        // std::cout << "workspace " << (state_machine__->workspace().state + workspace__*_robot.rate()).transpose() << std::endl;


        // std::cout << "desired frame " << desired__.transpose() << std::endl;
        // std::cout << "world frame " << world__.transpose() << std::endl;
        // std::cout << "worldJacobian frame " << state_machine__->worldJacobian() << std::endl;
        // std::cout << "shape_joint__ " << shape_joint__->get().transpose() << std::endl;


        // std::cout << "desired__\t" << desired__.transpose() << std::endl;
        // std::cout << "b_des";
        // for(int i = 0; i < 4; i++){
        //    __last_steer[i] =  std::atan2(world__[2*i+1], world__[2*i]);
        //    // mwoibn::eigen_utils::wrapToPi(__last_steer[i]);
        //   logger.add("beta_des"+std::to_string(i), __last_steer[i]);
        //   // std::cout << "\t" << __last_steer[i];
        //  }
         // std::cout << std::endl;

        // shape_wheel__->log(logger);
        // shape_joint__->log(logger);
        // state_machine__->log(logger);
        // restore__->log(logger);

        // estimated__ =  shape__->margin();
        //
        // estimated__ =  shape__->marginJ().leftCols<12>()*shape__->points().getJacobian()*_robot.command.velocity.get() *_robot.rate();
        // estimated__ +=  shape__->marginJ().rightCols<2>()*_robot.centerOfMass().getJacobian().topRows<2>()*_robot.command.velocity.get()*_robot.rate();
        //
        // for(int i = 0; i < estimated__.size(); i++){
        // //   logger.add(std::string("margin_est")+std::to_string(i), estimated__[i]);
        //   logger.add(std::string("margin")+std::to_string(i), shape__->margin()[i]);
        // }

        // for(int i = 0; i < _tasks["BASE"]->getTaskSize(); i++){
        //   logger.add(std::string("base_error_") + std::to_string(i), _tasks["BASE"]->getError()[i]);
        //   logger.add(std::string("gain_") + std::to_string(i), std::dynamic_pointer_cast<mwoibn::hierarchical_control::actions::Compute>(_actions["BASE"])->gain()[i]);
        // }

        //std::cout << std::dynamic_pointer_cast<mwoibn::hierarchical_control::actions::Compute>(_actions["BASE"])->gain().transpose() << std::endl;

        // F =  \tau - (JH^{-1}J^T)^{-1}JH^{-1} ( "OVERALL_FORCE"- state.torque - dJdq)
        // F_des = M (\ddot m + K_p * (m_des - m)) - I save it an recompute after log? 0 to see how well it is tracked?
        // \ddot m = JH^{-1} ("OVERALL_FORCE" - dJdq)

}
