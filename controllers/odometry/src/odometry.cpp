#include <mgnss/odometry/odometry.h>

#include <iomanip>

mgnss::odometry::Odometry::Odometry(mwoibn::robot_class::Robot& robot,
                                    std::vector<std::string> names, double r)
    : _robot(robot), _wheels_ph("ROOT", _robot), _r(r)
{
  _ids.setConstant(names.size(), mwoibn::NON_EXISTING);
  _state.setZero(names.size());
  _error.setZero(names.size());
  _distance.setZero(names.size());

  _selector.setOnes(names.size()); // assume all legs in ground contact
  _contacts.setOnes(names.size()); // assume all legs in ground contact

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
  }

  _robot.state.get(_state, _ids, mwoibn::robot_class::INTERFACE::POSITION);

  _estimated =
      _wheels_ph.getFullStatesWorld(); // start without an error for now

  _previous_state = _state;

  update();
}

void mgnss::odometry::Odometry::update()
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

  for (int i = 0; i < _wheels_ph.size(); i++)
    _pelvis[i] = _estimated[i] - _wheels_ph.getPointStateWorld(i);
  //  _base.head(3) = _estimated[0] - _pelvis[0]; // relay only on the first leg



  _compute2(); // this seems to be the best

  _base.tail(3) =
      _robot.state.get(mwoibn::robot_class::INTERFACE::POSITION).segment<3>(3);
  _robot.command.set(_base, {0, 1, 2, 3, 4, 5},
                     mwoibn::robot_class::INTERFACE::POSITION);


  //  std::cout.precision(6);

  //  std::cout << std::fixed << "incoming\n" <<
  //  _robot.state.get(mwoibn::robot_class::INTERFACE::POSITION).head(6) <<
  //  "estimated\n" << _base << std::endl;
  //  for (int i = 0; i < _wheels_ph.size(); i++)
  //    std::cout << std::fixed << _estimated[i][0] - _pelvis[i][0] << "\t" <<
  //    _estimated[i][1] - _pelvis[i][1] << "\t" <<  _estimated[i][2] -
  //    _pelvis[i][2] << "\t|";
  //      std::cout << std::fixed << _estimated[0][0] - _pelvis[0][0] << "\t" <<
  //      _estimated[0][1] - _pelvis[0][1] << "\t" <<  _estimated[0][2] -
  //      _pelvis[0][2] << "\t|";
  //      std::cout << std::fixed << _estimated[0][0] << "\t" <<
  //      _estimated[0][1]<< "\t" <<  _estimated[0][2]  << "\t|";
  //      std::cout << std::fixed << _pelvis[0][0] << "\t" <<  _pelvis[0][1] <<
  //      "\t" <<  _pelvis[0][2] << "\t|";

  //  std::cout << "er\t\t es\t\t re\t\t id\t\t|" << std::endl;

  //  for (int i = 0; i < 4; i++)
  //  {
  //      std::cout << std::fixed << _error[i]
  //                << "\t";
  //      std::cout << std::fixed  << _previous_state[i] << "\t";
  //      std::cout << std::fixed  << _state[i] << "\t|";
  //      std::cout << std::fixed  << _ids[i] << "\t|";

  //      std::cout << std::endl;
  //  }

  //    std::cout << "1\t\t\t\t\t\t|"
  //              << "2\t\t\t\t\t\t|"
  //              << "3\t\t\t\t\t\t|"
  //              << "4\t\t\t\t\t\t" << std::endl;
  //    std::cout << "er\t\t es\t\t re\t\t| "
  //              << "er\t\t es\t\t re\t\t|"
  //              << "er\t\t es\t\t re\t\t|"
  //              << "er\t\t es\t\t re\t\t|" << std::endl;

  //    for (int j = 0; j < 3; j++)
  //    {
  //      for (int i = 0; i < 4; i++)
  //      {
  //        std::cout << std::fixed << _estimated[i][j] -
  //        _wheels_ph.getPointStateWorld(i)[j]
  //                  << "\t";
  //        std::cout << std::fixed  << _estimated[i][j] << "\t";
  //        std::cout << std::fixed  << _wheels_ph.getPointStateWorld(i)[j] <<
  //        "\t|";
  //      }
  //      std::cout << std::endl;
  //    }

  _previous_state.noalias() = _state;

  //  return _base;
}

// just chose the "median"
void mgnss::odometry::Odometry::_compute1()
{

  _distances();

  _base.head(3) = _pelvis[_min()];
}

// remove element by element
void mgnss::odometry::Odometry::_compute2()
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
void mgnss::odometry::Odometry::_compute3()
{

}

void mgnss::odometry::Odometry::_mad()
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


void mgnss::odometry::Odometry::_average()
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

void mgnss::odometry::Odometry::_distances()
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

      _distance[j] += _distance[i];
    }
  }
}

int mgnss::odometry::Odometry::_max()
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

int mgnss::odometry::Odometry::_min()
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
