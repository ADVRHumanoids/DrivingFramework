#include "mgnss/state_estimation/acceleration_test.h"

#include <iomanip>

mgnss::state_estimation::AccelerationTest::AccelerationTest(mwoibn::robot_class::Robot& robot)
        : mgnss::modules::Base(robot), _frame("arm1_7", robot.getModel(), robot.state), _acceleration(_frame)
{
}

mgnss::state_estimation::AccelerationTest::AccelerationTest(mwoibn::robot_class::Robot& robot,
                                                std::string config_file)
        : mgnss::modules::Base(robot), _frame("arm1_7", robot.getModel(), robot.state), _acceleration(_frame)
{

        YAML::Node config = mwoibn::robot_class::Robot::getConfig(config_file);

        if (!config["modules"])
                throw std::invalid_argument(
                              std::string("Couldn't find modules configurations."));
        if (!config["modules"]["acceleration_test"])
                throw std::invalid_argument(
                              std::string("Couldn't find ground_forces module configuration."));

        config = config["modules"]["acceleration_test"];

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
        _allocate();
}

void mgnss::state_estimation::AccelerationTest::_checkConfig(YAML::Node config){
}

void mgnss::state_estimation::AccelerationTest::_allocate(){
}


void mgnss::state_estimation::AccelerationTest::init(){
        update();
}

void mgnss::state_estimation::AccelerationTest::update()
{
  _robot.get();
  _robot.updateKinematics();
  std::cout << "jacobian\n" << _acceleration.getWorld() << std::endl;


}
