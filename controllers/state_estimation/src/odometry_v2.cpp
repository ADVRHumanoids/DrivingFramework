#include "mgnss/state_estimation/odometry_v2.h"

#include <iomanip>

mgnss::state_estimation::OdometryV2::OdometryV2(mwoibn::robot_class::Robot& robot,
                                                std::vector<std::string> names, double r)
        : mgnss::modules::Base(robot), _wheels_ph("ROOT", _robot), _r(r)
{
        _x << 1,0,0;
        _y << 0,1,0;
        _z << 0,0,1;

        _allocate(names);
        _filter_ptr.reset(new mwoibn::filters::IirSecondOrder(3, 1000, 1));
}

mgnss::state_estimation::OdometryV2::OdometryV2(mwoibn::robot_class::Robot& robot,
                                                std::string config_file)
        : mgnss::modules::Base(robot), _wheels_ph("ROOT", _robot)
{
        _x << 1,0,0;
        _y << 0,1,0;
        _z << 0,0,1;
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

mgnss::state_estimation::OdometryV2::OdometryV2(mwoibn::robot_class::Robot& robot,
                                                YAML::Node config)
        : mgnss::modules::Base(robot), _wheels_ph("ROOT", _robot)
{
        _x << 1,0,0;
        _y << 0,1,0;
        _z << 0,0,1;

        _checkConfig(config);
        _initConfig(config);
}

void mgnss::state_estimation::OdometryV2::_initConfig(YAML::Node config){

        std::vector<std::string> names = _robot.getLinks(config["chain"].as<std::string>());

        _r = config["wheel_radius"].as<double>();
        _allocate(names);
        _filter_ptr.reset(new mwoibn::filters::IirSecondOrder(3, config["filter"]["cut_off_frequency"].as<double>(), config["filter"]["damping"].as<double>()));


}
void mgnss::state_estimation::OdometryV2::_checkConfig(YAML::Node config){

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

        std::cout << "OdometryV2: read " << config["chain"].as<std::string>() << " chain." << std::endl;
        std::cout << "OdometryV2: filter damping " << config["filter"]["damping"].as<double>() << std::endl;
        std::cout << "OdometryV2: filter cut-off frequency " << config["filter"]["cut_off_frequency"].as<double>() << std::endl;
        std::cout << "OdometryV2: wheel radius " << config["wheel_radius"].as<double>() << std::endl;

}

void mgnss::state_estimation::OdometryV2::_allocate(std::vector<std::string> names){
        _ids.setConstant(names.size(), mwoibn::NON_EXISTING);
        _state.setZero(names.size());
        _error.setZero(names.size());
        _distance.setZero(names.size());
        _selector.setConstant(names.size(), true); // assume all legs in ground contact
        _contacts.setConstant(names.size(), true); // assume all legs in ground contact
        _previous_state.setZero(names.size());
        _twists.setZero(mwoibn::std_utils::factorial(names.size()-1));
        _sum_twists.setZero(_twists.size());
        _selector_th.setConstant(_twists.size(), true);
        __es_2.setZero(2);
        __st_2.setZero(2);
        _base_ids.setZero(3);

        for (const auto& name : names)
                _wheels_ph.addPoint(name);

        mwoibn::Vector3 axis, pelvis;
        // add a reference point on the pelvis to compute the

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
                                      std::string("OdometryV2: Couldn't find a link ") + names[i]);
                _ids[i] = dof[0];

                _contact_points.push_back(_wheels_ph.getPointStateWorld(i));
        }

        for(auto& state: _wheels_ph.getFullStatesWorld()){
           _estimated.push_back(state); // initialize add a current point
           _step.push_back(state);
        }
   }


void mgnss::state_estimation::OdometryV2::init(){

        _filter_ptr->computeCoeffs(_robot.rate());

        _robot.get();

        _robot.updateKinematics();
        _robot.state.position.get(_state, _ids);
        _base_pos = _robot.state.position.get().head<3>();

        _filter_ptr->reset(_base_pos);

        //std::cout << "OdometryV2 filter: initial state: " << _base_pos.transpose() << std::endl;

        for(int i = 0; i < _estimated.size(); i++) {
                _contact_points[i].noalias() = _wheels_ph.getPointStateWorld(i);
                _estimated[i].noalias() = _contact_points[i];
        }

        _previous_state.noalias() = _state;

        update();


}

void mgnss::state_estimation::OdometryV2::update()
{
        //Get wheels position
        _robot.state.position.get(_state, _ids);

        // Compute a difference
        _error.noalias() = _state - _previous_state;

        // which legs are in ground contact
        _selector.noalias() = _contacts;
        _removeTwist();

        _increment();


        _estimateTwist();

        // _twist_es = mwoibn::Quaternion::fromAxisAngle(_z, _base[5]);

        _poseEstimation();
        _filter();
        // for(int i = 0; i < _step.size(); i++)
        //         _step[i] = _wheels_ph.getPointStateWorld(i);

        mwoibn::Quaternion temp = _twist_es*_swing;
        _base.tail<3>() = temp.toMatrix().eulerAngles(0,1,2);
        // std::cout << "base" << _base.transpose() << std::endl;
        _robot.command.position.set(_base, {0, 1, 2, 3, 4, 5});

        _previous_state.noalias() = _state;
        //_end = std::chrono::high_resolution_clock::now();
        //  return _base;
}

void mgnss::state_estimation::OdometryV2::_removeTwist(){

        // For now, retrive the quaternion from eulerAngles

        _imu = mwoibn::Quaternion::fromAxisAngle(_x, _robot.state.position.get()[3]);
        _imu = _imu*mwoibn::Quaternion::fromAxisAngle(_y, _robot.state.position.get()[4]);
        _imu = _imu*mwoibn::Quaternion::fromAxisAngle(_z, _robot.state.position.get()[5]);

        // std::cout << _imu << std::endl;
        // remove the rotation ground ground component
        _twist_raw = _imu.twistSwing(_z,_swing);

        mwoibn::Position _euler;
        // get robot base eueler angles
        _euler = _swing.toMatrix().eulerAngles(0, 1, 2);

        // set _robot.base with _swing only
        _base_ids << 3, 4, 5;
        _robot.state.position.set(_euler, _base_ids);
        _robot.updateKinematics();   // this way the kinematics is updated twice, that is not good, this can be easily handled in the plugin

}

void mgnss::state_estimation::OdometryV2::_filter(){
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
}

void mgnss::state_estimation::OdometryV2::_increment(){
        for (int i = 0; i < _state.size(); i++)
        {
                // compute world position of the wheel axis
                _directions[i] = _wheels_ph.point(i).getRotationWorld().col(2); // z axis
                // std::cout << _directions[i].transpose() << std::endl;
                // compute the wheel direction of motion
                _directions[i] = _directions[i].cross(_axes[i]);

                _directions[i].normalize();
                // compute the distance travelled
                _error[i] *= _r;
                // estimate current contact point position
                _step[i].noalias() = _directions[i] * _error[i]; // in the world frame
                _contact_points[i].noalias() = _wheels_ph.getPointStateWorld(i); // in the

        }

}

void mgnss::state_estimation::OdometryV2::_applyTwist(){
        for (int i = 0; i < _state.size(); i++)
        {
                // compute world position of the wheel axis
                //_directions[i] = _twist_es.rotate(_directions[i]); // z axis
                _contact_points[i].noalias() = _twist_es.rotate(_contact_points[i]);
                // estimate current contact point position
                _estimated[i].noalias() += _twist_es.rotate(_step[i]);
        }

}

void mgnss::state_estimation::OdometryV2::_poseEstimation(){

        // estimate postion of each wheel
        _applyTwist();

        for (int i = 0; i < _wheels_ph.size(); i++) {
                // compute the distance between the contact point and base origin
                // estimate the state for each leg
                _pelvis[i].noalias() = _estimated[i] - _contact_points[i];
        }

        // choose the solution
        _compute2();               // this seems to be the best

}

void mgnss::state_estimation::OdometryV2::_estimateTwist()
{
        int k = 0;
        // std::cout << _selector.transpose() << std::endl;
        // std::cout << "error "<< _error.transpose() << std::endl;
        for(int i = 0; i < _contacts.size(); i++) {
                if (!_selector[i]) {
                        _twists.segment(k, _contacts.size()-i-1).setConstant(mwoibn::MAX_DOUBLE);
                        _selector_th.segment(k, _contacts.size()-i-1).setConstant(false);


                        k+=_contacts.size()-i-1;
                        continue;
                }
                //std::cout << "diff " << __es_2.transpose() << ", " << __st_2.transpose() << std::endl;
                // std::cout << "step "<< _step[i].transpose() << ", " << _contact_points[i].transpose() << std::endl;


                for(int j = i+1; j < _contacts.size(); j++) {
                        if(!_selector[j]) {
                                _selector_th[k] = false;
                                _twists[k] = mwoibn::MAX_DOUBLE;
                                k+=1;
                        }
                        else{
                                _selector_th[k] = true;
                                __es_2[0] = _estimated[i][0] - _estimated[j][0];
                                __es_2[1] = _estimated[i][1] - _estimated[j][1];
                                __st_2[0] = _contact_points[i][0] - _contact_points[j][0];
                                __st_2[1] = _contact_points[i][1] - _contact_points[j][1];
                                __st_2[0] -= _step[i][0] - _step[j][0];
                                __st_2[1] -= _step[i][1] - _step[j][1];

                                _twists[k] = std::atan2(__st_2[0]*__es_2[1] - __st_2[1]*__es_2[0], __st_2[1]*__es_2[1] + __st_2[0]*__es_2[0]);
                                // std::cout << "i, " << i << " j " << j << std::endl;
                                // std::cout << "norm " << __es_2.norm() << std::endl;
                                // std::cout << "diff " << __es_2.transpose() << ", " << __st_2.transpose() << std::endl;
                                k+=1;
                        }
                }

        }
        _angles();
        // std::cout << "_selector_th " << _selector_th.transpose() << std::endl;

        // std::cout << "_twists " << _twists.transpose() << std::endl;
        // std::cout << "_sum_twists " << _sum_twists.transpose() << std::endl;
        _computeTheta();

}

void mgnss::state_estimation::OdometryV2::_averageTheta()
{
        double twist = 0;
        // _base[5] = 0;
        for (int i = 0; i < _selector_th.size(); i++)
        {
                if (_selector_th[i])
                        twist += _twists[i];
        }

        twist = twist / _selector_th.count();
        _twist_es = mwoibn::Quaternion::fromAxisAngle(_z, twist);

        // std::cout << "_average" << _base[5] << std::endl;
}

void mgnss::state_estimation::OdometryV2::_computeTheta()
{
        if (_selector_th.count() < 3)
        {
                _averageTheta(); // return average no better option?
                return;
        }

        if (_selector_th.count() == 3)
        {
                _compute1Theta();
                return;
        }

        _angles();
        _selector_th[_max(_selector_th, _sum_twists)] = 0;

        _computeTheta();
}
void mgnss::state_estimation::OdometryV2::_compute1Theta()
{
        _angles();
        // std::cout << "_computeTheta" << _selector_th.transpose() << std::endl;
        // std::cout << "_computeTheta" << _base[5] << std::endl;
        // _base[5] = _twists[_min(_selector_th, _sum_twists)];
        _twist_es = mwoibn::Quaternion::fromAxisAngle(_z, _twists[_min(_selector_th, _sum_twists)]);
}
// just chose the "median"
void mgnss::state_estimation::OdometryV2::_compute1()
{

        _distances();

        _base.head(3) = _pelvis[_min(_selector, _distance)];
}

// remove element by element
void mgnss::state_estimation::OdometryV2::_compute2()
{
        if (_selector.count() < 3)
        {
                _average(); // return average no better option?
                return;
        }

        if (_selector.count() == 3)
        {
                _compute1();
                return;
        }

        _distances();
        _selector[_max(_selector, _distance)] = 0;

        _compute2();
}

// remove element by element
void mgnss::state_estimation::OdometryV2::_compute3()
{

}

void mgnss::state_estimation::OdometryV2::_mad()
{

        mwoibn::VectorN distanceSotr, madV, madVSort;
        distanceSotr = _distance;

        //order
        std::sort(distanceSotr.data(), distanceSotr.data()+distanceSotr.size()); // sort vector

        double median, mad;
        int sum = _selector.count(); // number of meaningfull elements
        if(_selector.count()%2) // odd values
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

        if(_selector.count()%2) // odd values
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


void mgnss::state_estimation::OdometryV2::_average()
{
        _base.head(3).setZero();
        for (int i = 0; i < _selector.size(); i++)
        {
                if (_selector[i])
                        _base.head(3) += _pelvis[i];
        }

        _base.head(3).noalias() = _base.head(3) / _selector.count();
}

// compute MAD - remove and average
void mgnss::state_estimation::OdometryV2::_angles()
{
        for (int i = 0; i < _sum_twists.size(); i++)
                _sum_twists[i] = (_selector_th[i]) ? 0 : mwoibn::MAX_DOUBLE;

        for (int i = 0; i < _sum_twists.size(); i++)
        {
                if (!_selector_th[i])
                        continue;
                for (int j = i; j < _sum_twists.size(); j++)
                {
                        if (!_selector_th[j])
                                continue;
                        double sum = std::fabs(_twists[i] - _twists[j]);
                        mwoibn::eigen_utils::wrapToPi(sum);
                        _sum_twists[i] += sum;
                        _sum_twists[j] += sum;
                }
        }
}

void mgnss::state_estimation::OdometryV2::_distances()
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
                        _distance[j] += (_pelvis[i] - _pelvis[j]).norm();
                }
        }
}

int mgnss::state_estimation::OdometryV2::_max(mwoibn::VectorBool& selector, const mwoibn::VectorN& distance)
{

        int id = -1;
        double value = -1;
        for (int i = 0; i < selector.size(); i++)
        {
                if (selector[i] && distance[i] > value)
                {
                        id = i;
                        value = distance[i];
                }
        }

        return id;
}

int mgnss::state_estimation::OdometryV2::_min(mwoibn::VectorBool& selector, const mwoibn::VectorN& distance)
{
        int id = -1;
        double value = mwoibn::MAX_DOUBLE;
        for (int i = 0; i < selector.size(); i++)
        {
                if (selector[i] && distance[i] < value)
                {
                        id = i;
                        value = distance[i];
                }
        }
        return id;
}

void mgnss::state_estimation::OdometryV2::startLog(mwoibn::common::Logger& logger){
        logger.addField("time", 0);
        logger.addField("q_raw_x", _twist_raw.x());
        logger.addField("q_raw_y", _twist_raw.y());
        logger.addField("q_raw_z", _twist_raw.z());
        logger.addField("q_raw_w", _twist_raw.w());
        logger.addField("q_es_x", _twist_es.x());
        logger.addField("q_es_y", _twist_es.y());
        logger.addField("q_es_z", _twist_es.z());
        logger.addField("q_es_w", _twist_es.w());
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
void mgnss::state_estimation::OdometryV2::log(mwoibn::common::Logger& logger, double time){
        logger.addEntry("time", time);
        logger.addEntry("q_raw_x", _twist_raw.x());
        logger.addEntry("q_raw_y", _twist_raw.y());
        logger.addEntry("q_raw_z", _twist_raw.z());
        logger.addEntry("q_raw_w", _twist_raw.w());
        logger.addEntry("q_es_x", _twist_es.x());
        logger.addEntry("q_es_y", _twist_es.y());
        logger.addEntry("q_es_z", _twist_es.z());
        logger.addEntry("q_es_w", _twist_es.w());
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
