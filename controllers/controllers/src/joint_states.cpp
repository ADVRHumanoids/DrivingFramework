#include <mgnss/controllers/joint_states.h>


  mgnss::controllers::JointStates::JointStates(mwoibn::robot_class::Robot& robot)
      : _robot(robot), _wheels("pelvis", robot)
  {


    _vel_map = _robot.selectors().get("wheels").which();
    _ankle_map = _robot.selectors().get("camber").which();
    _yaw_map = _robot.selectors().get("yaws").which();

    _velocity.setZero(_vel_map.size());
    _vel_sign.setOnes(_vel_map.size());
    _last_ankle.setZero(_ankle_map.size());
    _des_ankle.setZero(_ankle_map.size());

    _init_ankle.setZero(_ankle_map.size());

    for (auto link : _robot.getLinks("wheels"))
      _wheels.addPoint(link);

  }

  void mgnss::controllers::JointStates::init(){
      _position = _robot.state.get(mwoibn::robot_class::INTERFACE::POSITION);

      _robot.state.get(_des_ankle, _ankle_map, mwoibn::robot_class::INTERFACE::POSITION);
      _robot.state.get(_init_ankle, _ankle_map, mwoibn::robot_class::INTERFACE::POSITION);

      _pos_ref = _position;
      _vel_ref = _velocity;

      _robot.state.get(_last_ankle, _ankle_map, mwoibn::robot_class::INTERFACE::POSITION);
      _robot.command.set(_position, mwoibn::robot_class::INTERFACE::POSITION);

      _wheels_positions = _wheels.getFullStatesReference();

  }

  bool mgnss::controllers::JointStates::setFullPosition(std::string name)
  {
    if (!_robot.groupStates().isDefined(name)) return false;

    for(int i = 0; i < _pos_ref.size(); i++){
        if(_robot.groupStates().get(name).get()[i] != mwoibn::NON_EXISTING)
            _pos_ref[i] = _robot.groupStates().get(name).get()[i];
    }

    _init = false;
    return true;

  }

  bool mgnss::controllers::JointStates::setPosition(std::string name)
  {
    if (!_robot.groupStates().isDefined(name)) return false;
    _init = true;

    for(int i = 0; i < _pos_ref.size(); i++){
        if(_robot.groupStates().get(name).get()[i] != mwoibn::NON_EXISTING)
            _pos_ref[i] = _robot.groupStates().get(name).get()[i];
    }

    _wheels_positions = _wheels.getFullStatesReference();

    _robot.state.set(_pos_ref, mwoibn::robot_class::INTERFACE::POSITION);
    _robot.updateKinematics();

    for (int i = 0; i < _wheels.size(); i++)
    {
      _last = _wheels.getPointStateReference(i);
      _error = _last - _wheels_positions[i];

      std::cout << i << std::endl;
      std::cout << _error << std::endl;

    if(_error.norm() > 0.0001){
      _des_ankle[i] = std::atan2(_error[1], _error[0]);
      _pos_ref[_ankle_map[i]] = _des_ankle[i] - _position[_yaw_map[i]];

    }


     events::limit(_position[_ankle_map[i]], _pos_ref[_ankle_map[i]]);
    double ankle = _pos_ref[_ankle_map[i]];



    if ( ( ankle<
            _robot.lower_limits.get(
               _ankle_map[i] , mwoibn::robot_class::INTERFACE::POSITION)) ||
        ( ankle >
            _robot.upper_limits.get(_ankle_map[i],
                                    mwoibn::robot_class::INTERFACE::POSITION)))
    {
      _pos_ref[_ankle_map[i]] += 3.1415926;
      mwoibn::eigen_utils::wrapToPi(_pos_ref[_ankle_map[i]]);
      _vel_sign[i] = -1;
    }
    else
      _vel_sign[i] = 1;
    _init_ankle[i] = _pos_ref[_ankle_map[i]];
    //_des_ankle[i] = _pos_ref[_ankle_map[i]];
    std::cout  << _pos_ref[_ankle_map[i]] << std::endl;

    }


    return true;
    }

  //  bool setVelocity(double vel) {
  //    for(int i = 0; i < _vel_map.size(); i++){
  //      if(_vel_map[i])
  //      _vel_ref[i] =  vel; }
  //  }

  void mgnss::controllers::JointStates::update()
  {
    _robot.state.get(_last_ankle, _ankle_map, mwoibn::robot_class::INTERFACE::POSITION);
    /*
    if(_init && _ankle_map.size()){

      for (int k = 0; k < _ankle_map.size(); k++)
      {
        int i = _ankle_map[k];
        if (std::fabs(_pos_ref[i] - _position[i]) > _step)
        {
          if (_pos_ref[i] - _position[i] > 0)
            _position[i] += _step;
          else
            _position[i] -= _step;
        }
        else
          _position[i] = _pos_ref[i];

      }

      _velocity.setZero();
      if ((_last_ankle - _init_ankle).cwiseAbs().maxCoeff() < 0.0005)
        _init = false;

      return;
    }
	*/

    _last_position = _position;

//    for (int i = 0; i < _wheels.size(); i++)
//    {
//      double pr = _pos_ref[_ankle_map[i]];
//      _pos_ref[_ankle_map[i]] = _des_ankle[i] - _position[_yaw_map[i]];
//      events::limit(pr, _pos_ref[_ankle_map[i]]);
//      double ankle = _pos_ref[_ankle_map[i]];

//      if ( ( ankle<
//              _robot.lower_limits.get(
//                 _ankle_map[i] , mwoibn::robot_class::INTERFACE::POSITION)) ||
//          ( ankle >
//              _robot.upper_limits.get(_ankle_map[i],
//                                      mwoibn::robot_class::INTERFACE::POSITION)))
//      {
//        _pos_ref[_ankle_map[i]] += 3.1415926;
//        mwoibn::eigen_utils::wrapToPi(_pos_ref[_ankle_map[i]]);
//        _vel_sign[i] = -1;
//      }

//      else
//        _vel_sign[i] = 1;

//    }
    for (int i = 0; i < _position.size(); i++)
    {
      if(_pos_ref[i] == mwoibn::NON_EXISTING) continue;
      if (std::fabs(_pos_ref[i] - _position[i]) > _step)
      {
        if (_pos_ref[i] - _position[i] > 0)
          _position[i] += _step;
        else
          _position[i] -= _step;
      }
      else
        _position[i] = _pos_ref[i];
    }

	/*
    for (int i = 0; i < _wheels.size(); i++)
    {
      _last = _wheels_positions[i];
      _wheels_positions[i] = _wheels.getPointStateReference(i);
      _error = _wheels_positions[i] - _last;

      _velocity[i] = _vel_sign[i] * _error.norm() / _robot.rate();

    }
	 */
//    std::cout << _velocity.transpose() << std::endl;
  }

  void mgnss::controllers::JointStates::send()
  {
    _robot.command.set(_position, mwoibn::robot_class::INTERFACE::POSITION);

    _robot.command.set(_velocity, _vel_map,
                       mwoibn::robot_class::INTERFACE::VELOCITY);

    _robot.update();
  }


  bool mgnss::controllers::JointStates::setVelocity(std::string name, double vel){

    double dof = mwoibn::NON_EXISTING;
    try{
		if(_robot.getDof(name).size()){
			dof = _robot.getDof(name)[0];
		}
		else
			return false;
    }
    catch (const std::out_of_range& e){
        return false;
    }
    for(int i = 0; i < _vel_map.size(); i++){
        if (_vel_map[i] == dof){
            _velocity[i] = vel;
            return true;
        }
    }

    return false;
  }

    bool mgnss::controllers::JointStates::setPosition(std::string name, double pos){

    double dof;
    try{

		if(_robot.getDof(name).size()){

			dof = _robot.getDof(name)[0];
		}
		else{
		return false;
		}
    }
    catch (const std::out_of_range& e){

        return false;
    }

    _pos_ref[dof] = pos;
    return true;
  }