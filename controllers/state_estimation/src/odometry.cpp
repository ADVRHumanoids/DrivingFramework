#include "mgnss/state_estimation/odometry.h"

#include <iomanip>

mgnss::state_estimation::Odometry::Odometry(mwoibn::robot_class::Robot& robot,
                                            std::vector<std::string> names, double r)
        : mgnss::modules::Base(robot), _wheels_ph("ROOT", _robot), _r(r)
{
        _allocate(names);
        _filter_ptr.reset(new mwoibn::filters::IirSecondOrder(3, 1000, 1));
}

mgnss::state_estimation::Odometry::Odometry(mwoibn::robot_class::Robot& robot,
                                            std::string config_file)
        : mgnss::modules::Base(robot), _wheels_ph("ROOT", _robot)
{
        YAML::Node config = mwoibn::robot_class::Robot::getConfig(config_file);

        if (!config["modules"])
                throw std::invalid_argument(
                              std::string("Couldn't find modules configurations."));
        if (!config["modules"]["odometry"])
                throw std::invalid_argument(
                              std::string("Couldn't find odometry module configuration."));

        config = config["modules"]["odometry"];

        _checkConfig(config);
        _initConfig(config);
}

mgnss::state_estimation::Odometry::Odometry(mwoibn::robot_class::Robot& robot,
                                            YAML::Node config)
        : mgnss::modules::Base(robot), _wheels_ph("ROOT", _robot)
{
        _checkConfig(config);
        _initConfig(config);
}

void mgnss::state_estimation::Odometry::_initConfig(YAML::Node config){

        std::vector<std::string> names = _robot.getLinks(config["chain"].as<std::string>());

        _r = config["wheel_radius"].as<double>();
        _allocate(names);
        _filter_ptr.reset(new mwoibn::filters::IirSecondOrder(3, config["filter"]["cut_off_frequency"].as<double>(), config["filter"]["damping"].as<double>()));


}

void mgnss::state_estimation::Odometry::_checkConfig(YAML::Node config){

        if (!config["chain"])
                throw std::invalid_argument(
                              std::string("Please specify srdf chain for odometry plugin."));
        if (!config["filter"])
                throw std::invalid_argument(
                              std::string("Please specify filter parameters for odometry plugin."));
        if (!config["filter"]["damping"])
                throw std::invalid_argument(
                              std::string("Please specify filter damping for odometry plugin."));
        if (!config["filter"]["cut_off_frequency"])
                throw std::invalid_argument(
                              std::string("Please specify filter cut-off frequency for odometry plugin."));
        if (!config["wheel_radius"])
                throw std::invalid_argument(
                              std::string("Please specify wheel radius [m] for odometry plugin."));

        std::cout << "Odometry: read " << config["chain"].as<std::string>() << " chain." << std::endl;
        std::cout << "Odometry: filter damping " << config["filter"]["damping"].as<double>() << std::endl;
        std::cout << "Odometry: filter cut-off frequency " << config["filter"]["cut_off_frequency"].as<double>() << std::endl;
        std::cout << "Odometry: wheel radius " << config["wheel_radius"].as<double>() << std::endl;

}

void mgnss::state_estimation::Odometry::_allocate(std::vector<std::string> names){
        _ids.setConstant(names.size(), mwoibn::NON_EXISTING);
        _state.setZero(names.size());
        _error.setZero(names.size());
        _distance.setZero(names.size());
        _selector.setOnes(names.size()); // assume all legs in ground contact
        _contacts.setOnes(names.size()); // assume all legs in ground contact
        _previous_state.setZero(names.size());

        for (const auto& name : names)
                _wheels_ph.addPoint(name);

        mwoibn::Vector3 axis, pelvis;
        axis << 0, 0, 1; // for now assume flat ground
        pelvis = _robot.state.position.get().head(3);
        for (int i = 0; i < names.size(); i++)
        {
                _axes.push_back(axis);
                _pelvis.push_back(axis);

                _directions.push_back(_wheels_ph.point(i).getRotationWorld().col(2));

                mwoibn::VectorInt dof = _robot.getDof(names[i]);
                if (dof.size() == 0)
                        throw std::invalid_argument(
                                      std::string("Odometry: Couldn't find a link ") + names[i]);
                _ids[i] = dof[0];

                _contact_points.push_back(_wheels_ph.getPointStateWorld(i));
        }

        for (auto& state: _wheels_ph.getFullStatesWorld())
          _estimated.push_back(state); // start without an error for now
}


void mgnss::state_estimation::Odometry::init(){

        _filter_ptr->computeCoeffs(_robot.rate());

        //std::cout << "raw" << _robot.state.position.get().head<6>().transpose() << std::endl;

//    _robot.feedbacks.reset();

        //std::cout << "reset" << _robot.state.position.get().head<6>().transpose() << std::endl;

        _robot.get();

        //std::cout << "get" << _robot.state.position.get().head<6>().transpose() << std::endl;

        _robot.updateKinematics();
        _robot.state.position.get(_state, _ids);
        _base_pos = _robot.state.position.get().head<3>();

        _filter_ptr->reset(_base_pos);

        //std::cout << "Odometry filter: initial state: " << _base_pos.transpose() << std::endl;

        for(int i = 0; i < _estimated.size(); i++) {
                _contact_points[i].noalias() = _wheels_ph.getPointStateWorld(i);
                _estimated[i].noalias() = _contact_points[i];
        }

        _previous_state.noalias() = _state;

        update();


}

void mgnss::state_estimation::Odometry::update()
{
        //Get wheels position
        _robot.state.position.get(_state, _ids);

        // Compute a difference
        _error.noalias() = _state - _previous_state;

        // which legs are in ground contact
        _selector.noalias() = _contacts;

        // estimate postion of each wheel
        for (int i = 0; i < _state.size(); i++)
        {
                // compute world position of the wheel axis
                _directions[i] = _wheels_ph.point(i).getRotationWorld().col(2); // z axis

                // compute the wheel direction of motion
                _directions[i] = _directions[i].cross(_axes[i]);

                _directions[i].normalize();
                // compute the distance travelled
                _error[i] *= _r;
                // estimate current contact point position
                _estimated[i] += _directions[i] * _error[i];

        }

        for (int i = 0; i < _wheels_ph.size(); i++) {
                // compute the distance between the contact point and base origin
                _contact_points[i].noalias() = _wheels_ph.getPointStateWorld(i);
                // estimate the state for each leg
                _pelvis[i].noalias() = _estimated[i] - _contact_points[i];
        }

        // choose the solution
        _compute2(); // this seems to be the best

        // add imu reading
        _base.tail(3) =
                _robot.state.position.get().segment<3>(3);

        // clear estimation based on a final result
        for(int i = 0; i < _wheels_ph.size(); i++)
                _estimated[i].noalias() = _base.head<3>() + _contact_points[i];

        _base_pos = _base.head(3);
        _base_raw = _base;

        // filter the results
        _filter_ptr->update(_base_pos);

        _base.head(3) = _base_pos;

        _base_filtered = _base;
        _base_filtered.head<3>() = _base_pos;
        //std::cout << _base << std::endl;
        _robot.command.position.set(_base, {0, 1, 2, 3, 4, 5});

        _previous_state.noalias() = _state;
        //_end = std::chrono::high_resolution_clock::now();
        //  return _base;
}

// just chose the "median"
void mgnss::state_estimation::Odometry::_compute1()
{

        _distances();

        _base.head(3) = _pelvis[_min()];
}

// remove element by element
void mgnss::state_estimation::Odometry::_compute2()
{
        if (_selector.sum() < 3)
        {
                _average(); // return average no better option?
                return;
        }

        if (_selector.sum() == 3)
        {
                _compute1();
                return;
        }

        _distances();
        _selector[_max()] = 0;

        _compute2();
}

// remove element by element
void mgnss::state_estimation::Odometry::_compute3()
{

}

void mgnss::state_estimation::Odometry::_mad()
{

        mwoibn::VectorN distanceSotr, madV, madVSort;
        distanceSotr = _distance;

        //order
        std::sort(distanceSotr.data(), distanceSotr.data()+distanceSotr.size()); // sort vector

        double median, mad;
        int sum = _selector.sum(); // number of meaningfull elements
        if(_selector.sum()%2) // odd values
                median = distanceSotr[(int)sum/2];
        else // even
                median = distanceSotr[(int)sum/2 - 1] + distanceSotr[(int)sum/2];


        madV = mwoibn::VectorN::Zero(sum);

        for (int i = 0; i < sum; i++) {
                if (!_selector[i]) continue;
                madV[i] = std::fabs(distanceSotr[i] - median);
        }

        madVSort = madV;
        std::sort(madVSort.data(), madVSort.data()+madVSort.size()); // sort vector

        if(_selector.sum()%2) // odd values
                mad = madVSort[(int)sum/2];
        else // even
                mad = madVSort[(int)sum/2 - 1] + madVSort[(int)sum/2];

        mad = 1.4826 * mad * 3;

        for (int i = 0; i < sum; i++) {
                if (madV[i] > mad) {
                        for (int j = 0; j < _selector.size(); j++) {
                                if (std::fabs(_distance[j] - distanceSotr[i]) < mwoibn::EPS) {
                                        _selector(j) = 0; // remove outliers
                                }
                        }
                }
        }

        _average();
}


void mgnss::state_estimation::Odometry::_average()
{
        _base.head(3).setZero();
        for (int i = 0; i < _selector.size(); i++)
        {
                if (_selector[i])
                        _base.head(3) += _pelvis[i];
        }

        _base.head(3).noalias() = _base.head(3) / _selector.sum();
}

// compute MAD - remove and average

void mgnss::state_estimation::Odometry::_distances()
{

        for (int i = 0; i < _distance.size(); i++)
                _distance[i] = (_selector[i]) ? 0 : mwoibn::MAX_DOUBLE;

        for (int i = 0; i < _distance.size(); i++)
        {
                if (!_selector[i])
                        continue;
                for (int j = i; j < _distance.size(); j++)
                {
                        if (!_selector[j])
                                continue;
                        _distance[i] += (_pelvis[i] - _pelvis[j]).norm();
                        //     _distance[j] += _distance[i];
                        _distance[j] += (_pelvis[i] - _pelvis[j]).norm();
                }
        }
}

int mgnss::state_estimation::Odometry::_max()
{

        int id = -1;
        double value = -1;
        for (int i = 0; i < _selector.size(); i++)
        {
                if (_selector[i] && _distance[i] > value)
                {
                        id = i;
                        value = _distance[i];
                }
        }

        return id;
}

int mgnss::state_estimation::Odometry::_min()
{
        int id = -1;
        double value = mwoibn::MAX_DOUBLE;
        for (int i = 0; i < _selector.size(); i++)
        {
                if (_selector[i] && _distance[i] < value)
                {
                        id = i;
                        value = _distance[i];
                }
        }
        return id;
}

void mgnss::state_estimation::Odometry::startLog(mwoibn::common::Logger& logger){
        logger.addField("time", 0);
//  logger.addField("raw_x", getRaw()[0]);
//  logger.addField("raw_y", getRaw()[1]);
//  logger.addField("raw_z", getRaw()[2]);
//  logger.addField("raw_tx", getRaw()[3]);
//  logger.addField("raw_ty", getRaw()[4]);
//  logger.addField("raw_tz", getRaw()[5]);
//  logger.addField("fil_x", getFiltered()[0]);
//  logger.addField("fil_y", getFiltered()[1]);
//  logger.addField("fil_z", getFiltered()[2]);
//  logger.addField("fil_tx", getFiltered()[3]);
//  logger.addField("fil_ty", getFiltered()[4]);
//  logger.addField("fil_tz", getFiltered()[5]);
        logger.start();
}
void mgnss::state_estimation::Odometry::log(mwoibn::common::Logger& logger, double time){
        logger.addEntry("time", time);
//  logger.addEntry("raw_x", getRaw()[0]);
//  logger.addEntry("raw_y", getRaw()[1]);
//  logger.addEntry("raw_z", getRaw()[2]);
//  logger.addEntry("raw_tx", getRaw()[3]);
//  logger.addEntry("raw_ty", getRaw()[4]);
//  logger.addEntry("raw_tz", getRaw()[5]);
//  logger.addEntry("fil_x", getFiltered()[0]);
//  logger.addEntry("fil_y", getFiltered()[1]);
//  logger.addEntry("fil_z", getFiltered()[2]);
//  logger.addEntry("fil_tx", getFiltered()[3]);
//  logger.addEntry("fil_ty", getFiltered()[4]);
//  logger.addEntry("fil_tz", getFiltered()[5]);
        logger.write();
}
