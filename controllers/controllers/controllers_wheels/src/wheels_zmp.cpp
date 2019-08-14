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
#include <mgnss/higher_level/qp/constraints/joint_constraint_v2.h>

#include <mgnss/higher_level/qp/constraints/minimum_limit.h>
#include <mgnss/higher_level/qp/constraints/maximum_limit.h>
#include <mgnss/higher_level/qp/constraints/integrate.h>

#include <mwoibn/robot_points/handler.h>

void mgnss::controllers::WheelsZMP::compute()
{
        _com_ptr->update();

        _leg_tasks["CAMBER"].first.updateError();

        for(int i = 0; i < 4; i++){
          //_eigen_scalar.noalias() = _leg_tasks["CAMBER"].second[i].getJacobian()*_robot.command.velocity.get();
          _eigen_scalar.noalias() = _leg_tasks["CAMBER"].second[i].getJacobian()*_robot.state.velocity.get();
          _min_camber_limit[i] =  _eigen_scalar[0]-1e-5;
          _max_camber_limit[i] =  _min_camber_limit[i]+2*1e-5;
        }
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
        _temp_state = _robot.command.velocity.get() - _robot.state.velocity.get();
        _temp_state = _temp_state/_robot.rate();
        _robot.command.acceleration.set(_temp_state );
        _temp_state.noalias() = _dynamic_ptr->getInertia()*_robot.command.acceleration.get();
        _temp_state -= - _robot.state["BIAS_FORCE"].get();
        // _robot.command.torque.set(_temp_state);

        _temp_state = _robot.states[QR].velocity.get()*_robot.rate();
        _temp_state += _robot.state.position.get();
        _robot.states[QR].position.set(_temp_state);

        _temp_state = _robot.states[QR].velocity.get() - _robot.state.velocity.get();
        _temp_state = _temp_state/_robot.rate();
        _robot.states[QR].acceleration.set(_temp_state);

        _temp_state.noalias() = _dynamic_ptr->getInertia()*_robot.states[QR].acceleration.get();
        _temp_state -=  _robot.state["BIAS_FORCE"].get();
        _robot.states[QR].torque.set( _temp_state );

        _temp_state.noalias() = _dynamic_ptr->getInertia()*_robot.state.acceleration.get();
        _temp_state -= _robot.state["BIAS_FORCE"].get();
        _robot.state[ESTIMATED_TORQUES].set(_temp_state);

        // _temp_state.noalias() = _robot.command.torque.get();
        // _temp_state.noalias() += _robot.states[GAINS].position.get().cwiseProduct(_robot.state.position.get()-_robot.command.position.get());
        // _temp_state.noalias() -= _robot.states[GAINS].velocity.get().cwiseProduct(_robot.state.velocity.get());
        //
        // _robot.state[CONTROLLER_TORQUES].set(_temp_state);


        // std::cout << "wheels\t" << _robot.command.position.get().transpose() << std::endl;
        // _robot.command.torque.set( ( _robot.command.velocity.get() - _robot.state.velocity.get() )/_robot.rate() );
        // .states[QR].torque.set( ( _robot.states[QR].velocity.get() - _robot.state.velocity.get() )/_robot.rate() );

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
        //mwoibn::VectorN beta(4);
        //for(int i = 0; i <  _leg_tasks["STEERING"].second.size(); i++)
        //  beta[i] = _leg_tasks["STEERING"].second[i].getCurrent();

        state_machine__->update();

        _qr_wrappers["SHAPE"]->update();
        _qr_wrappers["SHAPE_JOINT"]->update();
        // _tasks["CONSTRAINTS"]->update();
        // _tasks["BASE"]->update();
        // _tasks["CAMBER"]->update();
        // mwoibn::VectorN init_steer(4);
        for(int i = 0; i < 4; i++)
          _temp_4[i] = _leg_tasks["STEERING"].second[i].getCurrent();
        // std::cout << "init steer\t" << init_steer.transpose() << std::endl;

        _steering_ref_ptr->set(_temp_4);
        _robot.command.position.set(_robot.state.position.get());

        //std::cout << _robot.command.position.get() << std::endl;
//        _qr_wrappers["SHAPE_WHEEL"]->update();
        // _qr_wrappers["SHAPE_JOINT"]->update();

        // std::cout << "LINKS" << std::endl;
        // for(auto& name: _robot.getLinks(mwoibn::eigen_utils::iota(_robot.getDofs()), false))
        //   std::cout << name << std::endl;

}

void mgnss::controllers::WheelsZMP::step(){
        // _steering_ptr->setVelocity(_modified_support);

        _position += _linear_vel  * _robot.rate();
        _heading  += _angular_vel[2] * _robot.rate();
        _heading  -= 6.28318531 * std::floor((_heading + 3.14159265) / 6.28318531); // limit -pi:pi
}

void mgnss::controllers::WheelsZMP::_allocate(){
        _log_name.reserve(1000);
        WheelsControllerExtend::_allocate();
        _temp_4.setZero(4);
        _eigen_scalar.setZero(1);
        _com_ref.setZero(2);
        __last_steer.setZero(4);
        _modified_support.setZero( _robot.getLinks("wheels").size()*3);

        _forces.setZero(_modified_support.size());
        _zero.setZero( _modified_support.size());
        _temp_state.setZero(_robot.getDofs());

        _shape_extend_ptr->equality.add(mgnss::higher_level::PreviousTask(*_tasks["CONSTRAINTS"], _ik_ptr->state.command));
        // _shape_extend_ptr->equality.add(mgnss::higher_level::PreviousTask(*_tasks["BASE"], _ik_ptr->state.command));
        _shape_extend_ptr->equality.add(mgnss::higher_level::PreviousTask(*_tasks["BASE"], _robot.state.velocity.get()));

        // _shape_extend_ptr->hard_inequality.add(mgnss::higher_level::constraints::MinimumLimit(_tasks["CAMBER"]->getJacobian(), -0.0001));
        // _shape_extend_ptr->hard_inequality.add(mgnss::higher_level::constraints::MaximumLimit(_tasks["CAMBER"]->getJacobian(),  0.0001));
        // _shape_extend_ptr->hard_inequality.add(mgnss::higher_level::constraints::MinimumLimit(_tasks["CASTER"]->getJacobian(), -0.1));
        // _shape_extend_ptr->hard_inequality.add(mgnss::higher_level::constraints::MaximumLimit(_tasks["CASTER"]->getJacobian(),  0.1));
        // _shape_extend_ptr->hard_inequality.add(mgnss::higher_level::JointConstraint(_robot, mwoibn::eigen_utils::iota(_robot.getDofs()), {"POSITION","VELOCITY"}));
        //_shape_extend_ptr->hard_inequality.add(mgnss::higher_level::constraints::JointConstraintV2(_robot, mwoibn::eigen_utils::iota(_robot.getDofs()), {"POSITION","VELOCITY", "TORQUE"}));
        _shape_extend_ptr->hard_inequality.add(mgnss::higher_level::constraints::JointConstraintV2(_robot, _robot.getDof(_robot.getLinks("lower_body" ) ), {"POSITION","VELOCITY", "TORQUE"}, *_dynamic_ptr));
        for(auto& link: _robot.getLinks("hips")){
          _soft_hip.push_back(std::tuple<int, mwoibn::Matrix, mwoibn::VectorN, mwoibn::VectorN, mwoibn::VectorN, mwoibn::VectorN>(_robot.getDof(link)[0], mwoibn::Matrix(1, _robot.getDofs()), mwoibn::VectorN::Zero(1), mwoibn::VectorN::Zero(1), mwoibn::VectorN::Zero(1), mwoibn::VectorN::Zero(1)));
          std::get<1>(_soft_hip.back()).setZero();
          std::get<1>(_soft_hip.back())(0,_robot.getDof(link)[0]) = 1;
          std::get<2>(_soft_hip.back())[0] = _robot.state.position.get()[std::get<0>(_soft_hip.back())];
          std::get<2>(_soft_hip.back())[0] = -_robot.state.position.get()[std::get<0>(_soft_hip.back())];
        }
        std::get<4>(_soft_hip[0])[0] =  1.0;
        std::get<5>(_soft_hip[1])[0] = -1.0;
        std::get<5>(_soft_hip[2])[0] = -1.0;
        std::get<4>(_soft_hip[3])[0] =  1.0;
        std::get<5>(_soft_hip[0])[0] = -1.9;
        std::get<4>(_soft_hip[1])[0] =  1.9;
        std::get<4>(_soft_hip[2])[0] =  1.9;
        std::get<5>(_soft_hip[3])[0] = -1.9;
                //
//        _shape_extend_ptr->addSoft(mgnss::higher_level::constraints::Integrate(
//                                   mgnss::higher_level::constraints::MaximumLimit(std::get<1>(_soft_hip[0]), std::get<4>(_soft_hip[0])), _robot.rate(), std::get<2>(_soft_hip[0])), 1e3);
//        _shape_extend_ptr->addSoft(mgnss::higher_level::constraints::Integrate(
//                                   mgnss::higher_level::constraints::MinimumLimit(std::get<1>(_soft_hip[1]), std::get<5>(_soft_hip[1])), _robot.rate(), std::get<2>(_soft_hip[1])), 1e3);
//        _shape_extend_ptr->addSoft(mgnss::higher_level::constraints::Integrate(
//                                   mgnss::higher_level::constraints::MinimumLimit(std::get<1>(_soft_hip[2]), std::get<5>(_soft_hip[2])), _robot.rate(), std::get<2>(_soft_hip[2])), 1e3);
//        _shape_extend_ptr->addSoft(mgnss::higher_level::constraints::Integrate(
//                                   mgnss::higher_level::constraints::MaximumLimit(std::get<1>(_soft_hip[3]), std::get<4>(_soft_hip[3])), _robot.rate(), std::get<2>(_soft_hip[3])), 1e3);
//
//        _shape_extend_ptr->addSoft(mgnss::higher_level::constraints::Integrate(
//                                   mgnss::higher_level::constraints::MinimumLimit(std::get<1>(_soft_hip[0]), std::get<5>(_soft_hip[0])), _robot.rate(), std::get<3>(_soft_hip[0])), 1e3);
//        _shape_extend_ptr->addSoft(mgnss::higher_level::constraints::Integrate(
//                                   mgnss::higher_level::constraints::MaximumLimit(std::get<1>(_soft_hip[1]), std::get<4>(_soft_hip[1])), _robot.rate(), std::get<3>(_soft_hip[1])), 1e3);
//        _shape_extend_ptr->addSoft(mgnss::higher_level::constraints::Integrate(
//                                   mgnss::higher_level::constraints::MaximumLimit(std::get<1>(_soft_hip[2]), std::get<4>(_soft_hip[2])), _robot.rate(), std::get<3>(_soft_hip[2])), 1e3);
//        _shape_extend_ptr->addSoft(mgnss::higher_level::constraints::Integrate(
//                                   mgnss::higher_level::constraints::MinimumLimit(std::get<1>(_soft_hip[3]), std::get<5>(_soft_hip[3])), _robot.rate(), std::get<3>(_soft_hip[3])), 1e3);

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
        _robot.state.add(ESTIMATED_TORQUES, _robot.getDofs());
        _dynamic_ptr->subscribe({   mwoibn::dynamic_models::DYNAMIC_MODEL::INERTIA,
                                    mwoibn::dynamic_models::DYNAMIC_MODEL::INERTIA,
                                    mwoibn::dynamic_models::DYNAMIC_MODEL::INERTIA});
        // _robot.state.add(CONTROLLER_TORQUES, _robot.getDofs());

//        _robot.states.add(GAINS, _robot.getDofs());
//        mwoibn::VectorN temp_(_robot.getDofs());
//        temp_ << 0,0,0,0,0,0, 5000, 5000, 5000, 3000, 3000, 0, 5000, 5000, 5000, 3000, 3000, 0, 5000, 5000, 5000, 3000, 3000, 0,
//                 5000, 5000, 5000, 3000, 3000, 0, 2000, 500, 400, 400, 400, 300, 300, 200, 500, 400, 400, 400, 300, 300, 200;

//        _robot.states[GAINS].position.set(temp_);
//        temp_ << 0,0,0,0,0,0, 20, 20, 20, 20, 20, 30, 20, 20, 20, 20, 20, 30, 20, 20, 20, 20, 20, 30, 20, 20, 20, 20, 20, 30, 20, 5, 4, 4, 4, 3, 3, 2, 5, 4, 4, 4, 3, 3, 2;
//        _robot.states[GAINS].velocity.set(temp_);
}

void mgnss::controllers::WheelsZMP::_initIK(YAML::Node config){

    WheelsController::_initIK(config);


    YAML::Node steering = config["steerings"][config["steering"].as<std::string>()];
    std::cout << "Loaded sttering" << std::endl;
    for(auto entry : steering)
          std::cout << "\t" << entry.first << ": " << entry.second << std::endl;

    // _steering_ref_ptr.reset(new mgnss::higher_level::SteeringReactif(
    //           _robot, *_steering_ptr, _support_vel, steering["icm"].as<double>(), steering["sp"].as<double>(), steering["tracking"].as<double>(), _robot.rate(), steering["damp_icm"].as<double>(), steering["damp_sp"].as<double>(), steering["damp"].as<double>()));

    _steering_ref_ptr.reset(new mgnss::higher_level::SteeringShape(
                        _robot, *_steering_ptr, _support_vel, steering["icm"].as<double>(), steering["sp"].as<double>(), steering["tracking"].as<double>(), _robot.rate(), steering["damp_icm"].as<double>(), steering["damp_sp"].as<double>(), steering["damp"].as<double>()));

    // _steering_ref_ptr.reset(new mgnss::higher_level::Steering8(
    //                       _robot, *_steering_ptr, _support_vel, steering["icm"].as<double>(), steering["sp"].as<double>(), steering["tracking"].as<double>(), _robot.rate(), steering["damp_icm"].as<double>(), steering["damp_sp"].as<double>(), steering["damp"].as<double>()));

    shape_action__.reset(new mwoibn::hierarchical_control::actions::ShapeAction(*_shape_extend_ptr, *_steering_ptr,
     _leg_tasks["STEERING"].second, *_steering_ref_ptr, _leg_tasks["STEERING"].first, *_angles_ptr, _leg_tasks["CASTER"].first,
     _leg_tasks["CAMBER"].first, *state_machine__, _ik_ptr->state, _next_step, _robot.rate(), _robot));

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
    _min_camber_limit.setConstant(4, -1e-5); // 1e-6
    _max_camber_limit.setConstant(4,  1e-5); // 1e-6

    _shape_extend_ptr->hard_inequality.add(mgnss::higher_level::constraints::MinimumLimit(_angles_ptr->getJacobian(), _min_camber_limit)); // 1e-6
    _shape_extend_ptr->hard_inequality.add(mgnss::higher_level::constraints::MaximumLimit(_angles_ptr->getJacobian(), _max_camber_limit));

    _shape_extend_ptr->init();
    shape_action__->init();



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
                                config["track"].as<std::string>(), _robot, config, _world, "ROOT", tunning["COP"].as<double>()));
            state_machine__.reset(new mgnss::higher_level::StateMachine(_robot, config ));
            // mwoibn::VectorInt dofs__ = ;
            _qr_wrappers["SHAPE"] = std::unique_ptr<mgnss::higher_level::SupportShapingV4>(new mgnss::higher_level::SupportShapingV4(_robot, config, state_machine__->steeringFrames(), state_machine__->margin(), state_machine__->workspace()));
            _qr_wrappers["SHAPE_WHEEL"] = std::unique_ptr<mgnss::higher_level::QRJointSpaceV2>(new mgnss::higher_level::QRJointSpaceV2(*_qr_wrappers["SHAPE"], state_machine__->cost_I.jacobian.get(), state_machine__->cost_I.offset.get(), _robot, 0 ));
            _qr_wrappers["SHAPE_JOINT"] = std::unique_ptr<mgnss::higher_level::QRJointSpaceV2>(new mgnss::higher_level::QRJointSpaceV2(*_qr_wrappers["SHAPE_WHEEL"], state_machine__->cost_II.jacobian.get(), state_machine__->cost_II.offset.get(), _robot, 1e-7 ));
            // _qr_wrappers["SHAPE_JOINT"] = std::unique_ptr<mgnss::higher_level::QRJointSpaceV2>(new mgnss::higher_level::QRJointSpaceV2(*_qr_wrappers["SHAPE"], state_machine__->cost_I.jacobian.get(), state_machine__->cost_I.offset.get(), _robot ));


            _shape_extend_ptr.reset(new mgnss::higher_level::QpAggravated(_robot.getDof(_robot.getLinks(config["chain"].as<std::string>()))));
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


    logger.add("time", time);
  //
    logger.add("th", _robot.state.position.get()[5]);
    logger.add("r_th", _heading);

  //   //
  //   _forces = _robot.contacts().getReactionForce();
          mwoibn::Vector3 temp_pos;
          _pelvis_position_ptr->points().point(0).getLinearWorld(temp_pos);
         for(int i = 0; i < 3; i++){
           _log_name = "com_";
           _char = char('x'+i);
           _log_name += _char;
          // logger.add(std::string("cop_") + char('x'+i), _robot.centerOfPressure().get()[i]);
          logger.add(_log_name, _robot.centerOfMass().get()[i]);
          //logger.add(_names[counter], getBaseReference()[i]);
          //logger.add(_names[counter], getBaseReference()[i]);
          _log_name = "r_base_";
          _log_name += _char;
          logger.add(_log_name,  _pelvis_position_ptr->getReference(0)[i]);

          // logger.add(_names[counter], _steering_ptr->base.get()[i]);
          // logger.add(_names[counter], _robot.state.position.get()[i]);
          _log_name = "base_";
          _log_name += _char;
          logger.add(_log_name, temp_pos[i]);
          _log_name = "e_base_";
          _log_name += _char;
          logger.add(_log_name,  _pelvis_position_ptr->getError()[i]);
          _log_name = "state_";
          _log_name += _char;
          logger.add(_log_name,  _robot.state.position.get()[i]);

  //
  //        for(int k = 0; k < 4; k++){
  //
  //          logger.add(_names[counter], _steering_ptr->getPointStateReference(k)[i]);
  //          logger.add(_names[counter], _steering_ptr->getReference()[k*3+i]);
  //        //   logger.add(_names[counter], _forces[k*3+i]);
  //        }
        }


  //
        for(int i = 0; i < 30; i++){
          _char = std::to_string(i);

          _log_name = "pos_des_";
          _log_name += _char;
            logger.add(_log_name, _robot.command.position.get()[i]);

            _log_name = "vel_des_";
            _log_name += _char;
            logger.add(_log_name, _robot.command.velocity.get()[i]);

  //          logger.add("acc_des_" + std::to_string(i), _robot.command.acceleration.get()[i]);

            _log_name = "tau_des_";
            _log_name += _char;
            logger.add(_log_name, _robot.command.torque.get()[i]);

            _log_name = "pos_";
            _log_name += _char;
            logger.add(_log_name, _robot.state.position.get()[i]);

            _log_name = "vel_";
            _log_name += _char;
            logger.add(_log_name, _robot.state.velocity.get()[i]);

            _log_name = "m_pos_";
            _log_name += _char;
            logger.add(_log_name, _robot.motor.position.get()[i]);
            // logger.add("m_vel_" + std::to_string(i), _robot.motor.velocity.get()[i]);
  //          logger.add("acc_" + std::to_string(i), _robot.state.acceleration.get()[i]);
            _log_name = "tau_";
            _log_name += _char;
            logger.add(_log_name, _robot.state.torque.get()[i]);
  //          logger.add("pos_ll_" + std::to_string(i), _robot.lower_limits.position.get()[i]);
  //          logger.add("vel_ll_" + std::to_string(i), _robot.lower_limits.velocity.get()[i]);
  //          logger.add("tau_ll_" + std::to_string(i), _robot.lower_limits.torque.get()[i]);
  //          logger.add("pos_ul_" + std::to_string(i), _robot.upper_limits.position.get()[i]);
  //          logger.add("vel_ul_" + std::to_string(i), _robot.upper_limits.velocity.get()[i]);
  //          logger.add("tau_ul_" + std::to_string(i), _robot.upper_limits.torque.get()[i]);
            _log_name = "bias_";
            _log_name += _char;
            logger.add(_log_name,  _robot.state["BIAS_FORCE"][i]);
        }


  //
        for(int i = 0; i < 30; i++){
          _log_name = "pos_qr_";
          _log_name += _char;
            logger.add(_log_name, _robot.states[QR].position.get()[i]);

            _log_name = "vel_qr_";
            _log_name += _char;
            logger.add(_log_name, _robot.states[QR].velocity.get()[i]);

            _log_name = "tau_qr_";
            _log_name += _char;
            logger.add(_log_name, _robot.states[QR].torque.get()[i]);

            _log_name = "acc_qr_";
            _log_name += _char;
            logger.add(_log_name, _robot.states[QR].acceleration.get()[i]);
  //
        }
  //
        for(int i = 0; i < 4; i++){
            _eigen_scalar.noalias() = _leg_tasks["CAMBER"].second[i].getJacobian()*_robot.state.velocity.get();
            logger.add("camber_", _eigen_scalar[0]);
            _eigen_scalar.noalias() = _leg_tasks["CAMBER"].second[i].getJacobian()*_robot.states[QR].velocity.get();
            logger.add("camber_qr_", _eigen_scalar[0]);
            _eigen_scalar.noalias() = _leg_tasks["CAMBER"].second[i].getJacobian()*_robot.command.velocity.get();
            logger.add("camber_des_", _eigen_scalar[0]);
            logger.add("camber_err_", (-30*_leg_tasks["CAMBER"].first.getError()[i]));
            // logger.add(_names[counter], (_steering_ptr->getJacobian().row(i)*_robot.command.velocity.get())[0] );
            // logger.add(_names[counter], (_steering_ptr->getJacobian().row(i)*_robot.state.velocity.get())[0] );

          }




  //
  //        // for(int i = 0; i < 4 ; i++){
  //        //     logger.add("st_icm_" + std::to_string(i), _steering_ref_ptr->getICM()[i]);
  //        //     logger.add("st_sp_" + std::to_string(i), _steering_ref_ptr->getSP()[i]);
  //        //     logger.add("r_st_" + std::to_string(i), _steering_ref_ptr->get()[i]);
  //        //     logger.add("v_icm_" + std::to_string(i), _steering_ref_ptr->vICM()[i]);
  //        //     logger.add("v_sp_" + std::to_string(i), _steering_ref_ptr->vSP()[i]);
  //        //     logger.add("v_" + std::to_string(i), _steering_ref_ptr->v()[i]);
  //        //     logger.add("tan_sp_" + std::to_string(i), _steering_ref_ptr->getDampingSP()[i]);
  //        //     logger.add("tan_icm_" + std::to_string(i), _steering_ref_ptr->getDampingICM()[i]);
  //        //     logger.add("tan_" + std::to_string(i), _steering_ref_ptr->damp()[i]);
  //        // }
  //



          shape_action__->log(logger);
          state_machine__->log(logger);

  //        for(auto task: _tasks) std::cout << task.first << "\t" << task.second->getError().transpose() << std::endl;



}
