#include <mgnss/state_estimation/odometry.h>

#include <iomanip>

mgnss::state_estimation::Odometry::Odometry(mwoibn::robot_class::Robot& robot,
                                    std::vector<std::string> names, double r)
    : mgnss::modules::Base(robot), _wheels_ph("ROOT", _robot), _r(r)
{
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
  pelvis = _robot.state.get(mwoibn::robot_class::INTERFACE::POSITION).head(3);
  for (int i = 0; i < names.size(); i++)
  {
    _axes.push_back(axis);
    _pelvis.push_back(axis);

    _directions.push_back(_wheels_ph.point(i)
                              .getRotationWorld(_robot.state.get(
                                  mwoibn::robot_class::INTERFACE::POSITION))
                              .row(2));

    mwoibn::VectorInt dof = _robot.getDof(names[i]);
    if (dof.size() == 0)
      throw std::invalid_argument(
          std::string("Odometry: Couldn't find a link ") + names[i]);
    _ids[i] = dof[0];

    _contact_points.push_back(_wheels_ph.getPointStateWorld(i));
  }

  _estimated =
      _wheels_ph.getFullStatesWorld(); // start without an error for now

  _filter_ptr.reset(new mwoibn::filters::IirSecondOrder(3, 200, 1));

}

void mgnss::state_estimation::Odometry::init(){

    _filter_ptr->computeCoeffs(_robot.rate());

    //std::cout << "raw" << _robot.state.get().head<6>().transpose() << std::endl;

    _robot.feedbacks.reset();

    //std::cout << "reset" << _robot.state.get().head<6>().transpose() << std::endl;

    _robot.get();

    //std::cout << "get" << _robot.state.get().head<6>().transpose() << std::endl;

    _robot.updateKinematics();
    _robot.state.get(_state, _ids, mwoibn::robot_class::INTERFACE::POSITION);
    _base_pos = _robot.state.get(mwoibn::robot_class::INTERFACE::POSITION).head<3>();

    _filter_ptr->reset(_base_pos);

    //std::cout << "Odometry filter: initial state: " << _base_pos.transpose() << std::endl;

    for(int i = 0; i < _estimated.size(); i++){
        _contact_points[i] = _wheels_ph.getPointStateWorld(i);
        _estimated[i] = _contact_points[i];
    }

    _previous_state.noalias() = _state;

    update();


}

void mgnss::state_estimation::Odometry::update()
{

  _robot.state.get(_state, _ids, mwoibn::robot_class::INTERFACE::POSITION);

  _error.noalias() = _state - _previous_state;

  _selector.noalias() = _contacts;
  // estimate postion of each wheel
  for (int i = 0; i < _state.size(); i++)
  {
    _directions[i] = _wheels_ph.point(i)
                         .getRotationWorld(_robot.state.get(
                             mwoibn::robot_class::INTERFACE::POSITION))
                         .row(2); // z axis

    _directions[i] = _directions[i].cross(_axes[i]); //?
    _directions[i].normalize();

    _error[i] *= _r;

    _estimated[i] += _directions[i] * _error[i];

  }

  for (int i = 0; i < _wheels_ph.size(); i++){
    _contact_points[i] = _wheels_ph.getPointStateWorld(i);
    _pelvis[i] = _estimated[i] - _contact_points[i];
  }

  _compute2(); // this seems to be the best

  _base.tail(3) =
      _robot.state.get(mwoibn::robot_class::INTERFACE::POSITION).segment<3>(3);

  for(int i = 0; i < _wheels_ph.size(); i++)
    _estimated[i] = _base.head<3>() + _contact_points[i];

  _base_pos = _base.head(3);

  _base_raw = _base;

  _filter_ptr->update(_base_pos);

  _base.head(3) = _base_pos;

  _base_filtered = _base;
  _base_filtered.head<3>() = _base_pos;
  //std::cout << _base << std::endl;
  _robot.command.set(_base, {0, 1, 2, 3, 4, 5},
                     mwoibn::robot_class::INTERFACE::POSITION);

  _previous_state.noalias() = _state;

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

  for (int i = 0; i < sum; i++){
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

  for (int i = 0; i < sum; i++){
      if (madV[i] > mad){
        for (int j = 0; j < _selector.size(); j++){
          if (std::fabs(_distance[j] - distanceSotr[i]) < mwoibn::EPS){
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
