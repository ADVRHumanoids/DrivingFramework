#include "mgnss/state_estimation/acceleration_test.h"

#include <iomanip>

mgnss::state_estimation::AccelerationTest::AccelerationTest(mwoibn::robot_class::Robot& robot)
        : mgnss::modules::Base(robot), _frame("arm1_7", robot.getModel(), robot.state), _acceleration(_frame)
{
          _filter_ptr.reset(new mwoibn::filters::IirSecondOrder(_robot.state.acceleration.size(), 1000, 1));
}

mgnss::state_estimation::AccelerationTest::AccelerationTest(mwoibn::robot_class::Robot& robot, std::string config_file, std::string name)
        : mgnss::modules::Base(robot), _frame("arm1_7", robot.getModel(), robot.state), _acceleration(_frame)
{

        YAML::Node config = mwoibn::robot_class::Robot::getConfig(config_file);

        if (!config["modules"])
                throw std::invalid_argument(
                              std::string("Couldn't find modules configurations."));
        if (!config["modules"][name])
                throw std::invalid_argument(
                              std::string("Couldn't find ground_forces module configuration."));

        config = config["modules"][name];
        config["name"] = name;
        _checkConfig(config);
        _initConfig(config);
}

mgnss::state_estimation::AccelerationTest::AccelerationTest(mwoibn::robot_class::Robot& robot,
                                                YAML::Node config)
        : mgnss::modules::Base(robot), _frame("arm1_7", robot.getModel(), robot.state), _acceleration(_frame)
{
        _checkConfig(config);
        _initConfig(config);
}

void mgnss::state_estimation::AccelerationTest::_initConfig(YAML::Node config){
  _name = config["name"].as<std::string>();
  _filter_ptr.reset(new mwoibn::filters::IirSecondOrder(_robot.state.acceleration.size(), config["filter"]["cut_off_frequency"].as<double>(), config["filter"]["damping"].as<double>()));
  _allocate();
}

void mgnss::state_estimation::AccelerationTest::_checkConfig(YAML::Node config){
}

void mgnss::state_estimation::AccelerationTest::_allocate(){

  _vel_p.setZero(_robot.state.velocity.size());
  _acc_est.setZero(_robot.state.acceleration.size());
  //_f_acc_est.setZero(_robot.state.acceleration.size());
  _filter_ptr->reset(_acc_est);
  _point_acc.setZero(3);
}


void mgnss::state_estimation::AccelerationTest::init(){
        _filter_ptr->computeCoeffs(_robot.rate());

        update();
}

void mgnss::state_estimation::AccelerationTest::update()
{
  _robot.get();

  _acc_est.noalias() = _robot.state.velocity.get() - _vel_p;
  _acc_est.noalias() = _acc_est/_robot.rate();
  //_f_acc_est.noalias() = _acc_est;

  _filter_ptr->update(_acc_est);
  _robot.state.acceleration.set(_acc_est);

  _robot.updateKinematics();

  _point_acc = _acceleration.getWorld();
}

void mgnss::state_estimation::AccelerationTest::initLog(mwoibn::common::Logger& logger){
        logger.addField("time", 0);

        logger.addField("x", _point_acc[0]);
        logger.addField("y", _point_acc[1]);
        logger.addField("z", _point_acc[2]);
        // logger.addField("unfil_5", _acc_est[5]);
        // logger.addField("unfil_6", _acc_est[6]);
        // logger.addField("unfil_7", _acc_est[7]);
        // logger.addField("unfil_8", _acc_est[8]);
        //
        // logger.addField("fil_5", _f_acc_est[5]);
        // logger.addField("fil_6", _f_acc_est[6]);
        // logger.addField("fil_7", _f_acc_est[7]);
        // logger.addField("fil_8", _f_acc_est[8]);

}
void mgnss::state_estimation::AccelerationTest::log(mwoibn::common::Logger& logger, double time){
        logger.addEntry("time", time);

        logger.addEntry("x", _point_acc[0]);
        logger.addEntry("y", _point_acc[1]);
        logger.addEntry("z", _point_acc[2]);

        // logger.addEntry("unfil_5", _acc_est[5]);
        // logger.addEntry("unfil_6", _acc_est[6]);
        // logger.addEntry("unfil_7", _acc_est[7]);
        // logger.addEntry("unfil_8", _acc_est[8]);
        //
        // logger.addEntry("fil_5", _f_acc_est[5]);
        // logger.addEntry("fil_6", _f_acc_est[6]);
        // logger.addEntry("fil_7", _f_acc_est[7]);
        // logger.addEntry("fil_8", _f_acc_est[8]);

}
