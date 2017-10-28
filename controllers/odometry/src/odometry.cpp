#include <mgnss/odometry/odometry.h>

#include <iomanip>

mgnss::odometry::Odometry::Odometry(mwoibn::robot_class::Robot& robot,
                                    std::vector<std::string> names, double r)
    : _robot(robot), _wheels_ph("ROOT", _robot), _r(r)
{
  _ids.setConstant(names.size(), mwoibn::NON_EXISTING);
  _state.setZero(names.size());
  _error.setZero(names.size());

  for (const auto& name: names)
    _wheels_ph.addPoint(name);

  // getOffstes -- shouldn't this be a default function?

//  mwoibn::VectorN zeros = mwoibn::VectorN::Zero((_robot.getDofs()));
//  mwoibn::VectorN get_state =
//      _robot.state.get(mwoibn::robot_class::INTERFACE::POSITION);

//  _robot.state.set(zeros);
//  _robot.updateKinematics();

//  mwoibn::Vector7 offset = mwoibn::Vector7::Zero();

//  for (int i = 0; i < _robot.getDofs(); i++){
//    mwoibn::Quaternion::toVector(_wheels_ph.point(i).getOrientationWorld(zeros).transposed(), offset, 3);

//    _wheels_ph.setPointStateFixed(i, offset);
//  }

//  _robot.state.set(get_state);
//  _robot.updateKinematics();

  for (int i = 0; i < names.size(); i++)
  {
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
  //  std::cout <<  _state << std::endl;

  _error.noalias() = _state - _previous_state;

  // estimate postion of each wheel
  for (int i = 0; i < _state.size(); i++)
  {
    _directions[i] = _wheels_ph.point(i)
                         .getRotationWorld(_robot.state.get(
                             mwoibn::robot_class::INTERFACE::POSITION))
                         .row(2); // z axis
    mwoibn::Vector3 axis;
    axis << 0, 0, 1; // for now assume flat ground

    _directions[i] = _directions[i].cross(axis);  //?
    _directions[i].normalize();

    _error[i] *= _r;
    _estimated[i] += _directions[i] * _error[i];
  }

  // input should be 0,0,0 for floating base position
  // then the current wheel position should give the relative postion therefore the pelvis is estimated-wheel_position

  _pelvis = _wheels_ph.getFullStatesWorld();
  _base.head(3) = _estimated[0] - _pelvis[0]; // relay only on the first leg
  _base.tail(3) = _robot.state.get(mwoibn::robot_class::INTERFACE::POSITION).segment<3>(3);
  _robot.command.set(_base, {0,1,2,3,4,5}, mwoibn::robot_class::INTERFACE::POSITION);
//  std::cout.precision(6);

//  std::cout << std::fixed << "incoming\n" << _robot.state.get(mwoibn::robot_class::INTERFACE::POSITION).head(6) << "estimated\n" << _base << std::endl;
//  for (int i = 0; i < _wheels_ph.size(); i++)
//    std::cout << std::fixed << _estimated[i][0] - _pelvis[i][0] << "\t" <<  _estimated[i][1] - _pelvis[i][1] << "\t" <<  _estimated[i][2] - _pelvis[i][2] << "\t|";
//      std::cout << std::fixed << _estimated[0][0] - _pelvis[0][0] << "\t" <<  _estimated[0][1] - _pelvis[0][1] << "\t" <<  _estimated[0][2] - _pelvis[0][2] << "\t|";
//      std::cout << std::fixed << _estimated[0][0] << "\t" <<  _estimated[0][1]<< "\t" <<  _estimated[0][2]  << "\t|";
//      std::cout << std::fixed << _pelvis[0][0] << "\t" <<  _pelvis[0][1] << "\t" <<  _pelvis[0][2] << "\t|";

//  std::cout << _robot.state.get(mwoibn::robot_class::INTERFACE::POSITION).head(6) << std::endl;
//  std::cout << std::fixed << _robot.state.get()[0] << "\t" << _robot.state.get()[1] << "\t" << _robot.state.get()[2] << "\t|";

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
