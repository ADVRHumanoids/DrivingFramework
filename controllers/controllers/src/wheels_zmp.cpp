#include "mgnss/controllers/wheels_zmp.h"
#include "mgnss/higher_level/steering_v8.h"

//#include <mwoibn/hierarchical_control/tasks/contact_point_zmp.h>
#include <mgnss/controllers/devel/contact_point_zmp.h>

#include <mwoibn/hierarchical_control/tasks/contact_point_3D_rbdl_task.h>

#include <mwoibn/robot_points/torus_model.h>

#include <mwoibn/hierarchical_control/controllers/default.h>
#include <mwoibn/robot_points/handler.h>

void mgnss::controllers::WheelsZMP::compute()
{
        _com_ptr->update();
        //
        // if(!_steering_select.all()){
        //   _steering_ptr->updateState();
        //   _steering_ptr->update();
        // }
        // if(_steering_select.any()){
        //   _steering_ptr_2->updateState();
        //   _steering_ptr_2->update();
        // }
        // std::cout << _robot.centerOfPressure().get().transpose() << std::endl;

        // std::cout << "ZMP\t" << _steering_ptr_2->getError().transpose() << std::endl;
        // std::cout << "COM\t" << _steering_ptr->getError().transpose() << std::endl;
        // std::cout << "jZMP\t" << _steering_ptr_2->getJacobian() << std::endl;
        // std::cout << "jCOM\t" << _steering_ptr->getJacobian() << std::endl;

        WheelsControllerExtend::compute();
}

void mgnss::controllers::WheelsZMP::steering()
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


void mgnss::controllers::WheelsZMP::_setInitialConditions(){

  if(!_steering_select.all())
      _steering_ptr->reset();

  if(_steering_select.any())
        _steering_ptr_2->reset();

        _dt = _robot.rate();

        _leg_tasks["STEERING"].first.updateError();
        _leg_tasks["CAMBER"].first.updateError();
        _leg_tasks["CASTER"].first.updateError();

  // if(!_steering_select.all())
  //       _steering_ptr->updateState();
  // if(_steering_select.any())
  //       _steering_ptr_2->updateState();


        for(int i = 0, k = 0; i < _steering_select.size(); i++){
          _support.segment<3>(3*i) = (_steering_select[i])? _steering_ptr_2->getReference(k): _steering_ptr->getReference(i-k);
          k += _steering_select[i];
        }
        _support_vel.setZero();

        for(auto& item_: _leg_tasks)
            for(auto& angle_: item_.second.second)
                  angle_.reset();

        WheelsController::_setInitialConditions();

        _pelvis_position_ptr->points().point(0).getLinearWorld(_position);
        _position.head<2>() = _robot.centerOfMass().get().head<2>();
        _pelvis_position_ptr->setReference(0, _position);
        _com_ptr->setReference(_position);

        if(_steering_select.any())
              _steering_ptr_2->start();
}


void mgnss::controllers::WheelsZMP::_allocate(){
          WheelsControllerExtend::_allocate();
        _com_ref.setZero(2);

}

void mgnss::controllers::WheelsZMP::_initIK(YAML::Node config){

    WheelsController::_initIK(config);

    YAML::Node steering = config["steerings"][config["steering"].as<std::string>()];

    _steering_ref_ptr.reset(new mgnss::higher_level::Steering8(
                          _robot, *_steering_ptr_2, _support_vel, steering["icm"].as<double>(), steering["sp"].as<double>(), steering["tracking"].as<double>(), _robot.rate(), steering["damp_icm"].as<double>(), steering["damp_sp"].as<double>(), steering["damp"].as<double>()));
}

void mgnss::controllers::WheelsZMP::_createTasks(YAML::Node config){

        _name = config["name"].as<std::string>();

        WheelsControllerExtend::_createTasks(config);

        mwoibn::Vector3 pelvis;
        pelvis << 0, 0, 1;
        mwoibn::point_handling::PositionsHandler pelvis_ph("ROOT", _robot,
                                                           _robot.getLinks("base"));
        _pelvis_position_ptr.reset(
                new mwoibn::hierarchical_control::tasks::CartesianSelective(pelvis_ph,
                                                                            pelvis));

        _steering_select.setConstant(4,false);

        _com_ptr.reset(new mwoibn::hierarchical_control::tasks::CenterOfMass(_robot));
        _com_ptr->setDofs(_robot.selectors().get("lower_body").getBool());

        std::vector<std::string> names = {"wheel_1", "wheel_2", "wheel_3", "wheel_4"};

        mwoibn::robot_points::Handler<mwoibn::robot_points::TorusModel> contact_points(_robot.getDofs());

        for(auto& contact: _robot.contacts())
        {
            std::string name = _robot.getBodyName(contact->wrench().getBodyId());
            if(!std::count(names.begin(), names.end(), name)){
              std::cout << "Tracked point " << name << " could not be initialized" << std::endl;
              names.erase(std::remove(names.begin(), names.end(), name), names.end());
              continue;
            }
            _steering_select[contact_points.size()] = true;

            mwoibn::robot_points::TorusModel torus_(
                               _robot, mwoibn::point_handling::FramePlus(name,
                               _robot.getModel(), _robot.state),
                               mwoibn::Axis(config["reference_axis"][name]["x"].as<double>(),
                                            config["reference_axis"][name]["y"].as<double>(),
                                            config["reference_axis"][name]["z"].as<double>()),
                                            config["minor_axis"].as<double>(), config["major_axis"].as<double>(),
                                            contact->getGroundNormal());
            contact_points.add(torus_);

        }

          _contact_point.reset(new mwoibn::hierarchical_control::tasks::Aggravated());

          //_base_ptr.reset(new mwoibn::robot_points::LinearPoint(_robot.getLinks("base")[0], _robot));

          if(_steering_select.count() < 4){
            _steering_ptr.reset(
                  new mwoibn::hierarchical_control::tasks::ContactPoint3DRbdl({}, _robot, config, _robot.centerOfMass(), _robot.getLinks("base")[0]));
            _steering_ptr->subscribe(true, true, false);
            _contact_point->addTask(*_steering_ptr);
          }
          if(_steering_select.count()){
            _steering_ptr_2.reset(new mwoibn::hierarchical_control::tasks::ContactPointZMP(contact_points,
                                _robot, "pelvis", config["position_gain"].as<double>()));
            _steering_ptr_2->subscribe(true, true, false);
            _contact_point->addTask(*_steering_ptr_2);
          }

          _tasks["CONTACT_POINTS"] = _contact_point.get();
          _tasks["CONTACT_POINTS_1"] = _steering_ptr.get();
          _tasks["CONTACT_POINTS_2"] = _steering_ptr_2.get();

        _world_posture_ptr.reset(new mwoibn::hierarchical_control::tasks::Aggravated());

        _world_posture_ptr->addTask(*_pelvis_orientation_ptr);
        _world_posture_ptr->addTask(*_com_ptr);

        mwoibn::VectorBool select(3);
        select << false, false, true;
        _world_posture_ptr->addTask(*_pelvis_position_ptr, select);

        _tasks["BASE_GROUND"] = _com_ptr.get();
        _tasks["BASE_GRAVITY"] = _pelvis_position_ptr.get();
        _tasks["BASE"] = _world_posture_ptr.get();

}




void mgnss::controllers::WheelsZMP::log(mwoibn::common::Logger& logger, double time){
  // logger.add("com_x", getComFull()[0]);
  // logger.add("com_y", getComFull()[1]);

  // logger.add("r_com_x", refCom()[0]);
  // logger.add("r_com_y", refCom()[1]);

   logger.add("th", _robot.state.position.get()[5]);
   logger.add("r_th", _heading);
   //
       for(int i = 0; i < 3; i++){

         logger.add(std::string("cop_") + char('x'+i), _robot.centerOfPressure().get()[i]);
         logger.add(std::string("com_") + char('x'+i), _robot.centerOfMass().get()[i]);
         logger.add(std::string("r_base_") + char('x'+i), getBaseReference()[i]);

         for(int point = 0, k = 0; point < _steering_select.size(); point++){

           logger.add("cp_"   + std::to_string(point+1) + "_" + char('x'+i),
                           _steering_select[point] ? _steering_ptr_2->getPointStateReference(k)[i] : _steering_ptr->getPointStateReference(point-k)[i]);

           logger.add("r_cp_" + std::to_string(point+1) + "_" + char('x'+i),
                           _steering_select[point] ? _steering_ptr_2->getReference()[k*3+i] : _steering_ptr->getReference()[(point-k)*3+i]);

           // logger.add("position_error_" + std::to_string(point+1) + "_" + char('x'+i),
           //                 _steering_select[point] ? _steering_ptr_2->getPositionError()[k*3+i] : 0);

           logger.add("full_error_" + std::to_string(point+1) + "_" + char('x'+i),
                           _steering_select[point] ? _steering_ptr_2->getFullError()[k*3+i] : 0);

           logger.add("getForce_" + std::to_string(point+1) + "_" + char('x'+i),
                           _steering_select[point] ? _steering_ptr_2->getForce()[k*3+i] :0);

           // logger.add("com_error_" + std::to_string(point+1) + "_" + char('x'+i),
           //                _steering_select[point] ? _steering_ptr_2->getTestError()[k*3+i] : 0);

           k += _steering_select[point];

         }
       }

      for(auto& state_: {"MINUS_TORQUE","CONTACT_TORQUE","COM_TORQUE"}){
        for(int i = 0; i < _robot.state[state_].size(); i++)
          logger.add(state_ + std::string("_") + _robot.getLinks(i), _robot.state[state_][i]);
      }
      for(auto& state_: {"CONTACT_FORCE","COM_FORCE", "COM_CONTACT_FORCE"}){
          for(int i = 0; i < _robot.state[state_].size(); i++)
            logger.add(state_ + std::string("_") + std::to_string(i/3+1) + std::string("_") + char('x'+(i%3)), _robot.state[state_][i]);
      }
}
