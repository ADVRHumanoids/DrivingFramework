#include "mgnss/controllers/wheels_reactif.h"
#include "mgnss/higher_level/steering_reactif.h"
#include <mwoibn/hierarchical_control/tasks/cartesian_simplified_pelvis_task_v7.h>

mgnss::controllers::WheelsReactif::WheelsReactif(
        mwoibn::robot_class::Robot& robot, std::string config_file)
        : WheelsControllerExtend(robot)
{
        YAML::Node config = mwoibn::robot_class::Robot::getConfig(config_file)["modules"]["wheeled_motion"];
        std::cout << "wheels allocate" << std::endl;
        _x << 1, 0, 0;
        _y << 0, 1, 0;
        _y << 0, 1, 0;
        _z << 0, 0, 1;

        _createTasks(config);
        _initIK(config);
        _allocate();

        _steering_ref_ptr.reset(new mgnss::higher_level::SteeringReactif(
                                        _robot, *_steering_ptr, _support_vel, _test_steer, config["steer_open_loop"].as<double>(), config["steer_feedback"].as<double>(), config["tracking_gain"].as<double>(), _robot.rate(), config["damp_icm"].as<double>(), config["damp_sp"].as<double>(), config["steer_damp"].as<double>()));
}

mgnss::controllers::WheelsReactif::WheelsReactif(
        mwoibn::robot_class::Robot& robot, YAML::Node config)
        : WheelsControllerExtend(robot)
{
        _createTasks(config);
        _initIK(config);
        _allocate();

        _steering_ref_ptr.reset(new mgnss::higher_level::SteeringReactif(
                                        _robot, *_steering_ptr, _support_vel, _test_steer, config["steer_open_loop"].as<double>(), config["steer_feedback"].as<double>(), config["tracking_gain"].as<double>(), _robot.rate(), config["damp_icm"].as<double>(), config["damp_sp"].as<double>(), config["steer_damp"].as<double>()));
}


void mgnss::controllers::WheelsReactif::_allocate(){

        WheelsController::_allocate();

        _select_wheel = _robot.getDof(_robot.getLinks("wheels"));
        _start_steer.setZero(_select_steer.size());
        _resteer.setConstant(_select_steer.size(), false);
        _current_steer.setZero(_select_steer.size());
        _reset_count.setZero(_select_steer.size());
        _test_steer.setZero(_select_steer.size());

        _com_ref.setZero(2);

}

void mgnss::controllers::WheelsReactif::_initIK(YAML::Node config){
        std::cout << "Wheeled Motion loaded " << config["tunning"] << " tunning." << std::endl;

        if(!config["chain"])
                throw std::invalid_argument(std::string("Wheels Controller: configuration doesn't containt required filed 'chain'."));
        _select_ik = _robot.getDof(_robot.getLinks(config["chain"].as<std::string>()));


        config = config["tunnings"][config["tunning"].as<std::string>()];

        for(auto entry : config)
                std::cout << "\t" << entry.first << ": " << entry.second << std::endl;

        // int task = 0;
        double ratio = config["ratio"].as<double>(); // 4
        double damp = config["damping"].as<double>();
        // Set initaial HC tasks
        _hierarchical_controller_ptr->addTask(*_constraints_ptr, config["constraints"].as<double>(), damp);
        _hierarchical_controller_ptr->addTask(_leg_steer, config["leg_steer"].as<double>() * ratio, damp);


        mwoibn::VectorN gain_base(6);
        gain_base.head<3>() = mwoibn::VectorN::Constant(3, config["base_orinetation"].as<double>() * ratio);
        gain_base[3] = config["centre_of_mass_x"].as<double>() * ratio;
        gain_base[4] = config["centre_of_mass_y"].as<double>() * ratio;
        gain_base[5] = config["base_position"].as<double>() * ratio;
        _hierarchical_controller_ptr->addTask(*_world_posture_ptr, gain_base,  damp);

        _hierarchical_controller_ptr->addTask(*_steering_ptr, config["contact_point"].as<double>() * ratio, damp);

        _hierarchical_controller_ptr->addTask(_leg_camber, config["camber"].as<double>() * ratio, config["camber_damp"].as<double>());
        _hierarchical_controller_ptr->addTask(_leg_castor, config["castor"].as<double>() * ratio, config["castor_damp"].as<double>());

        _hierarchical_controller_ptr->update();

}

void mgnss::controllers::WheelsReactif::_createTasks(YAML::Node config){
        // Set-up hierachical controller
        _constraints_ptr.reset(
                new mwoibn::hierarchical_control::tasks::Constraints(_robot));
        mwoibn::Vector3 pelvis;
        pelvis << 0, 0, 1;
        mwoibn::point_handling::PositionsHandler pelvis_ph("ROOT", _robot,
                                                           _robot.getLinks("base"));
        _pelvis_position_ptr.reset(
                new mwoibn::hierarchical_control::tasks::CartesianSelective(pelvis_ph,
                                                                            pelvis));
        pelvis << 1, 1, 1;
        _pelvis_orientation_ptr.reset(
                new mwoibn::hierarchical_control::tasks::OrientationSelective(
                        mwoibn::point_handling::OrientationsHandler("ROOT", _robot,
                                                                    _robot.getLinks("base")),
                        pelvis, _robot));


        _com_ptr.reset(new mwoibn::hierarchical_control::tasks::CenterOfMass(_robot));

        _com_ptr->setDofs(_robot.selectors().get("lower_body").getBool());

        _steering_ptr.reset(
                new mwoibn::hierarchical_control::tasks::CartesianFlatReference4(
                        mwoibn::point_handling::PositionsHandler("ROOT", _robot,
                                                                 _robot.getLinks("wheels")),
                        _robot, *_com_ptr.get()));

        _createAngleTasks();

        _world_posture_ptr.reset(new mwoibn::hierarchical_control::tasks::Aggravated());

        _world_posture_ptr->addTask(*_pelvis_orientation_ptr);
        _world_posture_ptr->addTask(*_com_ptr);

        mwoibn::VectorBool select(3);
        select << false, false, true;
        _world_posture_ptr->addTask(*_pelvis_position_ptr, select);

}

void mgnss::controllers::WheelsReactif::init(){
        _robot.wait();
        _robot.get();
        _robot.updateKinematics();
        _robot.centerOfMass().update();

        _setInitialConditions();

}

void mgnss::controllers::WheelsReactif::_setInitialConditions(){

        WheelsControllerExtend::_setInitialConditions();

        _orientation = (mwoibn::Quaternion::fromAxisAngle(_y, _steering_ptr->getState()[4]) * mwoibn::Quaternion::fromAxisAngle(_x, _steering_ptr->getState()[5]));
        _heading = _steering_ptr->getState()[2];

        _pelvis_orientation_ptr->setReference(0, mwoibn::Quaternion::fromAxisAngle(_z, _heading)*(_orientation));

        _position = _pelvis_position_ptr->points().getPointStateWorld(0);
        _position.head<2>() = _robot.centerOfMass().get().head<2>();
        _pelvis_position_ptr->setReference(0, _position);
        _com_ptr->setReference(_position);
}



void mgnss::controllers::WheelsReactif::fullUpdate(const mwoibn::VectorN& support)
{
        _robot.get();
        _robot.updateKinematics();

        setSupport(support);
        update();

        _robot.send();
        _robot.wait();
}
void mgnss::controllers::WheelsReactif::compute()
{
        mgnss::controllers::WheelsController::compute();
        //_correct();


}

void mgnss::controllers::WheelsReactif::_correct(){

        _robot.state.get(_current_steer, _select_steer,
                         mwoibn::robot_class::INTERFACE::POSITION);
        _robot.command.get(_test_steer, _select_steer,
                           mwoibn::robot_class::INTERFACE::POSITION);

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
                        if(_reset_count[i] == 50) { //100
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
                        _test_steer[i] = _current_steer[i] + 3.5*mwoibn::PI/180;
                }
                else if ( _resteer[i] &&  _start_steer[i] > 0) {
                        _test_steer[i] = _current_steer[i] - 3.5*mwoibn::PI/180;
                }

        }


        _robot.command.set(_test_steer, _select_steer,
                           mwoibn::robot_class::INTERFACE::POSITION);
}

//void mgnss::controllers::WheelsReactif::_correct(){

//  _robot.state.get(_current_steer, _select_steer,
//                   mwoibn::robot_class::INTERFACE::POSITION);
//  _robot.command.get(_test_steer, _select_steer,
//                     mwoibn::robot_class::INTERFACE::POSITION);

//  // RESTEER AND CHECK FOR LIMITS
//  for (int i = 0; i < _test_steer.size(); i++)
//  {
//    double steer = _test_steer[i];

//    if ((_test_steer[i] < _l_limits[i] || _test_steer[i] > _u_limits[i]) && !_resteer[i])
//    {
//                                                                                                                                                                                                                                                                                                                                                                                      _start_steer[i] = _test_steer[i];
//      _resteer[i] = true;
////      std::cout << "WARNING: ankle yaw " << i << " on limit."
////                << std::endl; // NRT
//    }

//    if (_resteer[i])
//    {
////      std::cout << "dsgnkjhlfsymetljhmgdbfc" << std::endl;
//      mwoibn::eigen_utils::limitToHalfPi(_test_steer[i]);

//      if (std::fabs(_test_steer[i]) > 1.0 &&
//          std::fabs(_test_steer[i] - _start_steer[i]) < mwoibn::PI)
//      {
//        if (_start_steer[i] < 0)
//          _test_steer[i] += mwoibn::HALF_PI;
//        else
//          _test_steer[i] -= mwoibn::HALF_PI;
//      }

//    }

//    _resteer[i] =
//        _resteer[i] &&
//        (std::fabs(steer - _current_steer[i]) <
//         (std::fabs(_test_steer[i] - _current_steer[i]) + 0.1)) &&
//        std::fabs(_start_steer[i] - steer) < 2.5;

//    if( _resteer[i] && _start_steer[i] < 0){
//        _test_steer[i] = _current_steer[i] + 1*mwoibn::PI/180;
//    }
//    else if ( _resteer[i] &&  _start_steer[i] > 0){
//        _test_steer[i] = _current_steer[i] - 1*mwoibn::PI/180;
//    }

//  }

//  _robot.command.set(_test_steer, _select_steer,
//                     mwoibn::robot_class::INTERFACE::POSITION);

//}

void mgnss::controllers::WheelsReactif::steering()
{

        _steering_ref_ptr->compute(_next_step);

        steerings.noalias() = _steering_ref_ptr->get();

        for (int i = 0; i < 4; i++)
        {
                setSteering(i, steerings[i]);
        }

}

void mgnss::controllers::WheelsReactif::startLog(mwoibn::common::Logger& logger){
        logger.addField("time", 0.0);
/*
        logger.addField("e_base_z", getBaseError()[2]);
        logger.addField("r_base_z", getBodyPosition()[2]);

        logger.addField("e_base_rx", getBaseOrnError()[0]);
        logger.addField("e_base_ry", getBaseOrnError()[1]);
 */
        logger.addField("e_base_rz", getBaseOrnError()[2]);
//        logger.addField("base_rx", _robot.state.get()[3]);
//        logger.addField("base_ry", _robot.state.get()[4]);
        logger.addField("base_rz", _robot.state.get()[5]);
/*
        logger.addField("v_com_x", _linear_vel[0]);
        logger.addField("v_com_y", _linear_vel[1]);
        logger.addField("v_th", _angular_vel[2]);
 */
        logger.addField("com_x", getCom()[0]);
        logger.addField("com_y", getCom()[1]);

        logger.addField("r_com_x", refCom()[0]);
        logger.addField("r_com_y", refCom()[1]);

//  logger.addField("r_1", countResteer()[0]);
//  logger.addField("r_2", countResteer()[1]);
//  logger.addField("r_3", countResteer()[2]);
//  logger.addField("r_4", countResteer()[3]);

//  logger.addField("r_1", isResteer()[0]);
//  logger.addField("r_2", isResteer()[1]);
//  logger.addField("r_3", isResteer()[2]);
//  logger.addField("r_4", isResteer()[3]);

        logger.addField("cp_1_x", getCp(0)[0]);
        logger.addField("cp_1_y", getCp(0)[1]);
//        logger.addField("cp_1_z", getCp(0)[2]);
        logger.addField("cp_2_x", getCp(1)[0]);
        logger.addField("cp_2_y", getCp(1)[1]);
//        logger.addField("cp_2_z", getCp(1)[2]);
        logger.addField("cp_3_x", getCp(2)[0]);
        logger.addField("cp_3_y", getCp(2)[1]);
//        logger.addField("cp_3_z", getCp(2)[2]);
        logger.addField("cp_4_x", getCp(3)[0]);
        logger.addField("cp_4_y", getCp(3)[1]);
//        logger.addField("cp_4_z", getCp(3)[2]);

        logger.addField("r_cp_1_x", refCp()[0]);
        logger.addField("r_cp_1_y", refCp()[1]);
//        logger.addField("r_cp_1_z", refCp()[2]);
        logger.addField("r_cp_2_x", refCp()[3]);
        logger.addField("r_cp_2_y", refCp()[4]);
//        logger.addField("r_cp_2_z", refCp()[5]);
        logger.addField("r_cp_3_x", refCp()[6]);
        logger.addField("r_cp_3_y", refCp()[7]);
//        logger.addField("r_cp_3_z", refCp()[8]);
        logger.addField("r_cp_4_x", refCp()[9]);
        logger.addField("r_cp_4_y", refCp()[10]);
//        logger.addField("r_cp_4_z", refCp()[11]);


        logger.addField("st_icm_1", getSteerICM()[0]);
        logger.addField("st_icm_2", getSteerICM()[1]);
        logger.addField("st_icm_3", getSteerICM()[2]);
        logger.addField("st_icm_4", getSteerICM()[3]);


        logger.addField("st_sp_1", getSteerSP()[0]);
        logger.addField("st_sp_2", getSteerSP()[1]);
        logger.addField("st_sp_3", getSteerSP()[2]);
        logger.addField("st_sp_4", getSteerSP()[3]);
/*
        logger.addField("st_raw_1", rawSteer()[0]);
        logger.addField("st_raw_2", rawSteer()[1]);
 */
//        logger.addField("st_org_1", pureSteer()[0]);
//        logger.addField("st_org_2", pureSteer()[1]);
/*
        logger.addField("st_raw_3", rawSteer()[2]);
 */
//        logger.addField("st_org_3", pureSteer()[2]);
//        logger.addField("st_raw_4", rawSteer()[3]);*/
//        logger.addField("st_org_4", pureSteer()[3]);

        logger.addField("r_st_1", refSteer()[0]);
        logger.addField("r_st_2", refSteer()[1]);
        logger.addField("r_st_3", refSteer()[2]);
        logger.addField("r_st_4", refSteer()[3]);
        logger.addField("st_1", getSteer(0));
        logger.addField("st_2", getSteer(1));
        logger.addField("st_3", getSteer(2));
        logger.addField("st_4", getSteer(3));

        logger.addField("tan_sp_1", getDampingSP()[0]);
        logger.addField("tan_sp_2", getDampingSP()[1]);
        logger.addField("tan_sp_3", getDampingSP()[2]);
        logger.addField("tan_sp_4", getDampingSP()[3]);
        logger.addField("tan_icm_1", getDampingICM()[0]);
        logger.addField("tan_icm_2", getDampingICM()[1]);
        logger.addField("tan_icm_3", getDampingICM()[2]);
        logger.addField("tan_icm_4", getDampingICM()[3]);

/*
        logger.addField("v_icm_1", getVelICM()[0]);
        logger.addField("v_icm_2", getVelICM()[1]);
        logger.addField("v_icm_3", getVelICM()[2]);
        logger.addField("v_icm_4", getVelICM()[3]);

        logger.addField("v_sp_1", getVelSP()[0]);
        logger.addField("v_sp_2", getVelSP()[1]);
        logger.addField("v_sp_3", getVelSP()[2]);
        logger.addField("v_sp_4", getVelSP()[3]);

        logger.addField("v_1", getVel()[0]);
        logger.addField("v_2", getVel()[1]);
        logger.addField("v_3", getVel()[2]);
        logger.addField("v_4", getVel()[3]);
 */
        // logger.addField("d_1", getDamp()[0]);
        // logger.addField("d_2", getDamp()[1]);
//  logger.addField("ankle_yaw_1", _robot.state.get()[10]);
//  logger.addField("ankle_yaw_2", _robot.state.get()[16]);
//  logger.addField("ankle_yaw_3", _robot.state.get()[22]);
//  logger.addField("ankle_yaw_4", _robot.state.get()[28]);
//  logger.addField("e_st_1", errorSteer()[0]);
//  logger.addField("e_st_2", errorSteer()[1]);
//  logger.addField("e_st_3", errorSteer()[2]);
//  logger.addField("e_st_4", _controller_ptr->errorSteer()[3]);
        logger.start();
}

void mgnss::controllers::WheelsReactif::log(mwoibn::common::Logger& logger, double time){
        logger.addEntry("time", time);
//  logger.addEntry("state", _steering_ptr->getState()[2]);
//  logger.addEntry("twist", _steering_ptr->getTwist());

/*

   logger.addEntry("tref_0_0", test[0]);
   logger.addEntry("tref_0_1", test[1]);
   logger.addEntry("tref_0_2", test[2]);

 */
/*
       logger.addEntry("e_base_z", getBaseError()[2]);
       logger.addEntry("r_base_z", getBodyPosition()[2]);

       logger.addEntry("e_base_rx", getBaseOrnError()[0]);
       logger.addEntry("e_base_ry", getBaseOrnError()[1]);*/
        logger.addEntry("e_base_rz", getBaseOrnError()[2]);
//        logger.addEntry("base_rx", _robot.state.get()[3]);
//        logger.addEntry("base_ry", _robot.state.get()[4]);
        logger.addEntry("base_rz", _robot.state.get()[5]);
/*
   //  logger.addEntry("base_x", _robot.state.get()[0]);
   //  logger.addEntry("base_y", _robot.state.get()[1]);
   //  logger.addEntry("base_z", _robot.state.get()[2]);
   /*
        logger.addEntry("v_com_x", _linear_vel[0]);
        logger.addEntry("v_com_y", _linear_vel[1]);
        logger.addEntry("v_th", _angular_vel[2]);
 */
        logger.addEntry("com_x", getCom()[0]);
        logger.addEntry("com_y", getCom()[1]);

        logger.addEntry("r_com_x", refCom()[0]);
        logger.addEntry("r_com_y", refCom()[1]);

//  logger.addEntry("r_1", countResteer()[0]);
//  logger.addEntry("r_2", countResteer()[1]);
//  logger.addEntry("r_3", countResteer()[2]);
//  logger.addEntry("r_4", countResteer()[3]);

//  logger.addEntry("r_1", isResteer()[0]);
//  logger.addEntry("r_2", isResteer()[1]);
//  logger.addEntry("r_3", isResteer()[2]);
//  logger.addEntry("r_4", isResteer()[3]);

        logger.addEntry("cp_1_x", getCp(0)[0]);
        logger.addEntry("cp_1_y", getCp(0)[1]);
//        logger.addField("cp_1_z", getCp(0)[2]);
        logger.addEntry("cp_2_x", getCp(1)[0]);
        logger.addEntry("cp_2_y", getCp(1)[1]);
//        logger.addField("cp_2_z", getCp(1)[2]);
        logger.addEntry("cp_3_x", getCp(2)[0]);
        logger.addEntry("cp_3_y", getCp(2)[1]);
//        logger.addField("cp_3_z", getCp(2)[2]);
        logger.addEntry("cp_4_x", getCp(3)[0]);
        logger.addEntry("cp_4_y", getCp(3)[1]);
//        logger.addField("cp_4_z", getCp(3)[2]);

        logger.addEntry("r_cp_1_x", refCp()[0]);
        logger.addEntry("r_cp_1_y", refCp()[1]);
//        logger.addField("r_cp_1_z", refCp()[2]);
        logger.addEntry("r_cp_2_x", refCp()[3]);
        logger.addEntry("r_cp_2_y", refCp()[4]);
//        logger.addField("r_cp_2_z", refCp()[5]);
        logger.addEntry("r_cp_3_x", refCp()[6]);
        logger.addEntry("r_cp_3_y", refCp()[7]);
//        logger.addField("r_cp_3_z", refCp()[8]);
        logger.addEntry("r_cp_4_x", refCp()[9]);
        logger.addEntry("r_cp_4_y", refCp()[10]);


        logger.addEntry("st_icm_1", getSteerICM()[0]);
        logger.addEntry("st_icm_2", getSteerICM()[1]);
        logger.addEntry("st_icm_3", getSteerICM()[2]);
        logger.addEntry("st_icm_4", getSteerICM()[3]);


        logger.addEntry("st_sp_1", getSteerSP()[0]);
        logger.addEntry("st_sp_2", getSteerSP()[1]);
        logger.addEntry("st_sp_3", getSteerSP()[2]);
        logger.addEntry("st_sp_4", getSteerSP()[3]);
/*
        logger.addEntry("st_raw_1", rawSteer()[0]);
        logger.addEntry("st_raw_2", rawSteer()[1]);
 */
//        logger.addEntry("st_org_1", pureSteer()[0]);
//        logger.addEntry("st_org_2", pureSteer()[1]);
/*        logger.addEntry("st_raw_3", rawSteer()[2]);*/
//        logger.addEntry("st_org_3", pureSteer()[2]);
        /*logger.addEntry("st_raw_4", rawSteer()[3]);*/
//        logger.addEntry("st_org_4", pureSteer()[3]);

        logger.addEntry("r_st_1", refSteer()[0]);
        logger.addEntry("r_st_2", refSteer()[1]);
        logger.addEntry("r_st_3", refSteer()[2]);
        logger.addEntry("r_st_4", refSteer()[3]);
        logger.addEntry("st_1", getSteer(0));
        logger.addEntry("st_2", getSteer(1));
        logger.addEntry("st_3", getSteer(2));
        logger.addEntry("st_4", getSteer(3));

        logger.addEntry("tan_sp_1", getDampingSP()[0]);
        logger.addEntry("tan_sp_2", getDampingSP()[1]);
        logger.addEntry("tan_sp_3", getDampingSP()[2]);
        logger.addEntry("tan_sp_4", getDampingSP()[3]);
        logger.addEntry("tan_icm_1", getDampingICM()[0]);
        logger.addEntry("tan_icm_2", getDampingICM()[1]);
        logger.addEntry("tan_icm_3", getDampingICM()[2]);
        logger.addEntry("tan_icm_4", getDampingICM()[3]);

/*
        logger.addEntry("v_icm_1", getVelICM()[0]);
        logger.addEntry("v_icm_2", getVelICM()[1]);
        logger.addEntry("v_icm_3", getVelICM()[2]);
        logger.addEntry("v_icm_4", getVelICM()[3]);

        logger.addEntry("v_sp_1", getVelSP()[0]);
        logger.addEntry("v_sp_2", getVelSP()[1]);
        logger.addEntry("v_sp_3", getVelSP()[2]);
        logger.addEntry("v_sp_4", getVelSP()[3]);

        logger.addEntry("v_1", getVel()[0]);
        logger.addEntry("v_2", getVel()[1]);
        logger.addEntry("v_3", getVel()[2]);
        logger.addEntry("v_4", getVel()[3]);
 */
        // logger.addEntry("d_1", getDamp()[0]);
        // logger.addEntry("d_2", getDamp()[1]);
//  logger.addEntry("ankle_yaw_1", _robot.state.get()[10]);
//  logger.addEntry("ankle_yaw_2", _robot.state.get()[16]);
//  logger.addEntry("ankle_yaw_3", _robot.state.get()[22]);
//  logger.addEntry("ankle_yaw_4", _robot.state.get()[28]);
//  logger.addEntry("e_st_1", errorSteer()[0]);
//  logger.addEntry("e_st_2", errorSteer()[1]);
//  logger.addEntry("e_st_3", errorSteer()[2]);
//  logger.addEntry("e_st_4", _controller_ptr->errorSteer()[3]);

        logger.write();
}