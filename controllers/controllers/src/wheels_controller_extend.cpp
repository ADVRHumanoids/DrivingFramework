#include "mgnss/controllers/wheels_controller_extend.h"

mgnss::controllers::WheelsControllerExtend::WheelsControllerExtend(mwoibn::robot_class::Robot& robot)
        : WheelsController(robot)
{

}

void mgnss::controllers::WheelsControllerExtend::_create(YAML::Node config){
        _createTasks(config);
        _initIK(config);
        _allocate();
}


void mgnss::controllers::WheelsControllerExtend::resetSteering()
{
        for (auto& task_: _leg_tasks["STEERING"].second)
                task_.setReference(0);
}

void mgnss::controllers::WheelsControllerExtend::compute()
{
        mgnss::controllers::WheelsController::compute();
        _correct();

}

void mgnss::controllers::WheelsControllerExtend::_allocate(){

        WheelsController::_allocate();

        _start_steer.setZero(_select_steer.size());
        _resteer.setConstant(_select_steer.size(), false);
        _current_steer.setZero(_select_steer.size());
        _reset_count.setZero(_select_steer.size());
        _test_steer.setZero(_select_steer.size());

}



void mgnss::controllers::WheelsControllerExtend::_correct(){

          _robot.state.position.get(_current_steer, _select_steer);
          _robot.command.position.get(_test_steer, _select_steer);

          // RESTEER AND CHECK FOR LIMITS
          for (int i = 0; i < _test_steer.size(); i++)
          {
                  double steer = _test_steer[i];

                  if (((_test_steer[i] - _l_limits[i]) < -0.005 && (_current_steer[i] - _l_limits[i]) < 0.005)
                      || ((_test_steer[i] - _u_limits[i]) > 0.005  && (_current_steer[i] - _u_limits[i]) > -0.005))
                  {
                          if(!_resteer[i]) {
                                  _reset_count[i] = _reset_count[i] + 1;
                          }
                          if(_reset_count[i] == 20) { //100
                                  _resteer[i] = true;
                                  _start_steer[i] = _test_steer[i];
                                  _reset_count[i] = 0;
                          }
                  }
                  else{
                          _reset_count[i] = 0;
                  }


                  if (_resteer[i])
                  {
                          mwoibn::eigen_utils::limitToHalfPi(_test_steer[i]);

                          if (std::fabs(_test_steer[i]) > 1.0 &&
                              std::fabs(_test_steer[i] - _start_steer[i]) < mwoibn::PI)
                          {
                                  if (_start_steer[i] < 0)
                                          _test_steer[i] += mwoibn::HALF_PI;
                                  else
                                          _test_steer[i] -= mwoibn::HALF_PI;
                          }

                  }

                  _resteer[i] =
                          _resteer[i] &&
                          (std::fabs(steer - _current_steer[i]) <
                           (std::fabs(_test_steer[i] - _current_steer[i]) + 0.5)) &&
                          std::fabs(_start_steer[i] - steer) < 2.8; // I could add a wheel velocity condition here

                  if( _resteer[i] && _start_steer[i] < 0) {
                          _test_steer[i] = _current_steer[i] + 5.0*mwoibn::PI/180;
                  }
                  else if ( _resteer[i] &&  _start_steer[i] > 0) {
                          _test_steer[i] = _current_steer[i] - 5.0*mwoibn::PI/180;
                  }

          }
          //std::cout << _reset_count.transpose() << std::endl;


          _robot.command.position.set(_test_steer, _select_steer);
  }


void mgnss::controllers::WheelsControllerExtend::_createTasks(YAML::Node config){

        WheelsController::_createTasks(config);
        mwoibn::Axis ax_;

        for(auto& name_: {"STEERING", "CASTER", "CAMBER"})
                _leg_tasks[name_] = {mwoibn::hierarchical_control::tasks::Aggravated(), std::vector<mwoibn::hierarchical_control::tasks::Angle>{}};

        for(auto& name_: _robot.getLinks("wheels")){
          ax_  = mwoibn::Axis(config["reference_axis"][name_]["x"].as<double>(),
                             config["reference_axis"][name_]["y"].as<double>(),
                             config["reference_axis"][name_]["z"].as<double>());
          _leg_tasks["CAMBER"].second.push_back(mwoibn::hierarchical_control::tasks::Angle(
                     mwoibn::robot_class::angles::Camber(_robot,  mwoibn::point_handling::Frame(name_, _robot.getModel(), _robot.state), ax_), _robot));
          _leg_tasks["STEERING"].second.push_back(mwoibn::hierarchical_control::tasks::Angle(
                     mwoibn::robot_class::angles::Steering(_robot, mwoibn::point_handling::Frame(name_, _robot.getModel(), _robot.state), ax_), _robot));
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
}


void mgnss::controllers::WheelsControllerExtend::fullUpdate(const mwoibn::VectorN& support)
{
        _robot.get();
        _robot.updateKinematics();

        setSupport(support);
        update();

        _robot.send();
        _robot.wait();
}

void mgnss::controllers::WheelsControllerExtend::_setInitialConditions(){

        WheelsController::_setInitialConditions();

        _steering_ptr->reset();

        _dt = _robot.rate();

        for(auto& task_: _leg_tasks) task_.second.first.updateError();

        _steering_ptr->updateState();

        for(int i = 0; i < _leg_tasks["STEERING"].second.size(); i++)
                steerings[i] = _leg_tasks["STEERING"].second[i].getCurrent();

        _support.noalias() = _steering_ptr->getReference();
        _support_vel.setZero();

        for(auto& item_: _leg_tasks)
            for(auto& angle_: item_.second.second)
                  angle_.reset();

}


void mgnss::controllers::WheelsControllerExtend::steering()
{

        _steering_ref_ptr->compute(_next_step);

        steerings.noalias() = _steering_ref_ptr->get();

        for (int i = 0; i < 4; i++)
        {
                setSteering(i, steerings[i]);
        }

}

void mgnss::controllers::WheelsControllerExtend::initLog(mwoibn::common::Logger& logger){
        logger.addField("time", 0);
        //
        logger.addField("th", _robot.state.position.get()[5]);
        logger.addField("r_th", _heading);
        //

        for(int i = 0; i < 3; i++){
          for(int point = 0; point < 4; point++){
            logger.addField("cp_"   + std::to_string(point+1) + "_" + char('x'+i), getCp(point)[i]);
            logger.addField("r_cp_" + std::to_string(point+1) + "_" + char('x'+i), refCp()[point*3+i]);
            logger.addField("force_" + std::to_string(point+1) + "_" + char('x'+i), _steering_ptr->getForce()[3*point+i]);
          }
        }

}

void mgnss::controllers::WheelsControllerExtend::log(mwoibn::common::Logger& logger, double time){
   logger.addEntry("time", time);
   logger.addEntry("th", _robot.state.position.get()[5]);
   logger.addEntry("r_th", _heading);

   for(int i = 0; i < 3; i++){
       for(int point = 0; point < 4; point++){
           logger.addEntry("cp_"   + std::to_string(point+1) + "_" + char('x'+i), getCp(point)[i]);
           logger.addEntry("r_cp_" + std::to_string(point+1) + "_" + char('x'+i), refCp()[point*3+i]);
           logger.addEntry("force_" + std::to_string(point+1) + "_" + char('x'+i), _steering_ptr->getForce()[3*point+i]);
       }
   }
}
