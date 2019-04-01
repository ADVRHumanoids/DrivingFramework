#include "mgnss/controllers/wheels_zmp.h"
// #include "mgnss/higher_level/steering_v8.h"
#include "mgnss/higher_level/steering_v8.h"
#include "mgnss/higher_level/steering_reactif.h"
#include "mgnss/higher_level/steering_shape.h"

#include <mgnss/controllers/devel/contact_point_zmp_v2.h>

#include <mwoibn/hierarchical_control/tasks/contact_point_3D_rbdl_task.h>

#include <mwoibn/robot_points/torus_model.h>

#include <mwoibn/hierarchical_control/controllers/default.h>
#include <mgnss/higher_level/previous_task.h>
#include <mgnss/higher_level/qp/constraints/joint_constraint.h>
#include <mgnss/higher_level/qp/constraints/minimum_limit.h>
#include <mgnss/higher_level/qp/constraints/maximum_limit.h>
#include <mgnss/higher_level/qp/constraints/integrate.h>

#include <mwoibn/robot_points/handler.h>

void mgnss::controllers::WheelsZMP::compute()
{
        _com_ptr->update();
        //_support = _support + _support_vel*_robot.rate();
        // without resttering
        WheelsController::compute();
        // for(int i = 0; i < 4; i++){
          // double weight = std::fabs( _leg_tasks["CAMBER"].first.getTask(i).getJacobian()(0, 6*(i+1)+3) / _leg_tasks["CAMBER"].first.getTask(i).getJacobian()(0, 6*(i+1)+4)  );
          // double weight_2 = std::fabs( _tasks["CONTACT_POINTS_1"]->getJacobian()(2*i+1, 6*(i+1)+4) / _tasks["CONTACT_POINTS_1"]->getJacobian()(2*i+1, 6*(i+1)+0)  );

          // _qr_wrappers.back()->damping()[6*(i+1)] = 0.000002+std::fabs(_tasks["CONTACT_POINTS_1"]->getJacobian()(2*i+1, 6*(i+1)+4))*std::tanh(0.1*weight_2);

          // _leg_tasks["CAMBER"].first.setWeight(3.0, i);
          // _leg_tasks["CASTER"].first.setWeight(0.1*(1-std::tanh( 0.1*std::pow(weight,3) ) ), i);
          // _leg_tasks["CASTER"].first.setWeight(0.5, i);

          // std::cout << "weight\t" << weight_2 << std::endl;
          // std::cout << "weight\t" << _leg_tasks["CASTER"].first.getWeight(i);
          // std::cout << "\t" << _leg_tasks["CASTER"].first.getWeight(i) << std::endl;
        // }
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
        _qr_wrappers["SHAPE_JOINT"]->update();
        // _tasks["CONSTRAINTS"]->update();
        // _tasks["BASE"]->update();
        // _tasks["CAMBER"]->update();
        mwoibn::VectorN init_steer(4);
        for(int i = 0; i < 4; i++)
          init_steer[i] = _leg_tasks["STEERING"].second[i].getCurrent();
        std::cout << "init steer\t" << init_steer.transpose() << std::endl;

        _steering_shape_ptr->set(init_steer);
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


        _shape_extend_ptr->equality.add(mgnss::higher_level::PreviousTask(*_tasks["CONSTRAINTS"], _ik_ptr->state.command));
        _shape_extend_ptr->equality.add(mgnss::higher_level::PreviousTask(*_tasks["BASE"], _ik_ptr->state.command));

        // _shape_extend_ptr->hard_inequality.add(mgnss::higher_level::constraints::MinimumLimit(_tasks["CAMBER"]->getJacobian(), -0.0001));
        // _shape_extend_ptr->hard_inequality.add(mgnss::higher_level::constraints::MaximumLimit(_tasks["CAMBER"]->getJacobian(),  0.0001));
        // _shape_extend_ptr->hard_inequality.add(mgnss::higher_level::constraints::MinimumLimit(_tasks["CASTER"]->getJacobian(), -0.1));
        // _shape_extend_ptr->hard_inequality.add(mgnss::higher_level::constraints::MaximumLimit(_tasks["CASTER"]->getJacobian(),  0.1));
        _shape_extend_ptr->hard_inequality.add(mgnss::higher_level::JointConstraint(_robot, mwoibn::eigen_utils::iota(_robot.getDofs()), {"POSITION","VELOCITY"}));

        for(auto& link: _robot.getLinks("hips")){
          _soft_hip.push_back(mwoibn::Matrix(1, _robot.getDofs()));
          _soft_hip.back().setZero();
          _soft_hip.back()(0,_robot.getDof(link)[0]) = 1;
        }
        //
        _shape_extend_ptr->addSoft(mgnss::higher_level::constraints::Intergate(
                                   mgnss::higher_level::constraints::MaximumLimit(_soft_hip[0],  1.0), _robot.rate(), _robot.state.position), 1e3);
        _shape_extend_ptr->addSoft(mgnss::higher_level::constraints::Intergate(
                                   mgnss::higher_level::constraints::MinimumLimit(_soft_hip[1], -1.0), _robot.rate(), _robot.state.position), 1e3);
        _shape_extend_ptr->addSoft(mgnss::higher_level::constraints::Intergate(
                                   mgnss::higher_level::constraints::MinimumLimit(_soft_hip[2], -1.0), _robot.rate(), _robot.state.position), 1e3);
        _shape_extend_ptr->addSoft(mgnss::higher_level::constraints::Intergate(
                                   mgnss::higher_level::constraints::MaximumLimit(_soft_hip[3],  1.0), _robot.rate(), _robot.state.position), 1e3);

        _shape_extend_ptr->addSoft(mgnss::higher_level::constraints::Intergate(
                                   mgnss::higher_level::constraints::MinimumLimit(_soft_hip[0], -1.9), _robot.rate(), _robot.state.position), 1e3);
        _shape_extend_ptr->addSoft(mgnss::higher_level::constraints::Intergate(
                                   mgnss::higher_level::constraints::MaximumLimit(_soft_hip[1],  1.9), _robot.rate(), _robot.state.position), 1e3);
        _shape_extend_ptr->addSoft(mgnss::higher_level::constraints::Intergate(
                                   mgnss::higher_level::constraints::MaximumLimit(_soft_hip[2],  1.9), _robot.rate(), _robot.state.position), 1e3);
        _shape_extend_ptr->addSoft(mgnss::higher_level::constraints::Intergate(
                                   mgnss::higher_level::constraints::MinimumLimit(_soft_hip[3], -1.9), _robot.rate(), _robot.state.position), 1e3);

        // _qr_wrappers["SHAPE"]->init();
        // _qr_wrappers["SHAPE_WHEEL"]->init();
        _qr_wrappers["SHAPE_JOINT"]->init();
        // mwoibn::VectorInt dofs__ = _robot.getDof(_robot.getLinks("hips"));
        //
        // for(int i =0; i < dofs__.size(); i++)
        //   _qr_wrappers["SHAPE_JOINT"]->cost().quadratic(dofs__[i], dofs__[i]) = 1e1;
        //
        //
        // dofs__ = _robot.getDof(_robot.getLinks("ankle_yaws"));
        //
        // for(int i =0; i < dofs__.size(); i++)
        //     _qr_wrappers["SHAPE_JOINT"]->cost().quadratic(dofs__[i], dofs__[i]) = 1e-4;
}

void mgnss::controllers::WheelsZMP::_initIK(YAML::Node config){

    WheelsController::_initIK(config);


    YAML::Node steering = config["steerings"][config["steering"].as<std::string>()];
    std::cout << "Loaded sttering" << std::endl;
    for(auto entry : steering)
          std::cout << "\t" << entry.first << ": " << entry.second << std::endl;

    _steering_ref_ptr.reset(new mgnss::higher_level::SteeringReactif(
              _robot, *_steering_ptr, _support_vel, steering["icm"].as<double>(), steering["sp"].as<double>(), steering["tracking"].as<double>(), _robot.rate(), steering["damp_icm"].as<double>(), steering["damp_sp"].as<double>(), steering["damp"].as<double>()));

    _steering_shape_ptr.reset(new mgnss::higher_level::SteeringShape(
                        _robot, *_steering_ptr, _support_vel, steering["icm"].as<double>(), steering["sp"].as<double>(), steering["tracking"].as<double>(), _robot.rate(), steering["damp_icm"].as<double>(), steering["damp_sp"].as<double>(), steering["damp"].as<double>()));

    // _steering_ref_ptr.reset(new mgnss::higher_level::Steering8(
    //                       _robot, *_steering_ptr, _support_vel, steering["icm"].as<double>(), steering["sp"].as<double>(), steering["tracking"].as<double>(), _robot.rate(), steering["damp_icm"].as<double>(), steering["damp_sp"].as<double>(), steering["damp"].as<double>()));

    shape_action__.reset(new mwoibn::hierarchical_control::actions::ShapeAction(*_shape_extend_ptr, *_steering_ptr,
     _leg_tasks["STEERING"].second, *_steering_shape_ptr, _leg_tasks["STEERING"].first, *_angles_ptr, _leg_tasks["CASTER"].first,
     _leg_tasks["CAMBER"].first, *state_machine__, *_qr_wrappers["SHAPE"], _ik_ptr->state, _next_step, _robot.rate()));
    _ik_ptr->addAfter(*shape_action__, *_actions["BASE"]);

    YAML::Node tunning = config["tunnings"][config["tunning"].as<std::string>()];

    for(auto tune: tunning) std::cout << tune.first << std::endl;

    if(!_qr_wrappers["CASTER"]){
        mwoibn::VectorN gain;
        _readTask(tunning, "CASTER", gain);
       _qr_wrappers["CASTER"] = std::unique_ptr<mgnss::higher_level::QrTaskWrapper>(new mgnss::higher_level::QrTaskWrapper(*_tasks["CASTER"], gain, 0, _robot));
       _qr_wrappers["CASTER"]->init();
    }
    // _angles_ptr->addTask(*_tasks["CASTER"]); //  I was applying this constraint
    _angles_ptr->addTask(*_tasks["CAMBER"]);
    // _angles_ptr->init();
    // _shape_extend_ptr->add(*_qr_wrappers["CASTER"]);
    _shape_extend_ptr->hard_inequality.add(mgnss::higher_level::constraints::MinimumLimit(_angles_ptr->getJacobian(), -1e-6)); // 1e-6
    _shape_extend_ptr->hard_inequality.add(mgnss::higher_level::constraints::MaximumLimit(_angles_ptr->getJacobian(),  1e-6));

    _shape_extend_ptr->init();

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
            _qr_wrappers["SHAPE_WHEEL"] = std::unique_ptr<mgnss::higher_level::QRJointSpaceV2>(new mgnss::higher_level::QRJointSpaceV2(*_qr_wrappers["SHAPE"], state_machine__->stateJacobian(), state_machine__->stateOffset(), _robot, 0 ));
            _qr_wrappers["SHAPE_JOINT"] = std::unique_ptr<mgnss::higher_level::QRJointSpaceV2>(new mgnss::higher_level::QRJointSpaceV2(*_qr_wrappers["SHAPE_WHEEL"], state_machine__->wheelOrientation().passJacobian(), _zero, _robot, 1e-6 ));
            // _qr_wrappers["SHAPE_JOINT"] = std::unique_ptr<mgnss::higher_level::QRJointSpaceV2>(new mgnss::higher_level::QRJointSpaceV2(*_qr_wrappers["SHAPE"], state_machine__->stateJacobian(), state_machine__->stateOffset(), _robot ));


            _shape_extend_ptr.reset(new mgnss::higher_level::QpAggravated(_robot.getDofs()));
            _angles_ptr.reset(new mwoibn::hierarchical_control::tasks::Aggravated());
            _shape_extend_ptr->add(*_qr_wrappers["SHAPE_JOINT"]);

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

       std::cout << "wheels position\t" << _robot.state.position.get().head<6>().transpose() << std::endl;

      std::cout << "wheels velocity\t" << _robot.state.velocity.get().head<6>().transpose() << std::endl;

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
