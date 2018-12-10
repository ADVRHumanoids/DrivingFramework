#ifndef POINT_HANDLING_RAW_ORIENTATIONS_HANDLER_H
#define POINT_HANDLING_RAW_ORIENTATIONS_HANDLER_H

#include "mwoibn/point_handling/base_points_handler.h"
//#include "mwoibn/point_handling/frame.h"

namespace mwoibn
{

namespace point_handling
{

class RawOrientationsHandler : public BasePointsHandler<Orientation::O>
{
  typedef Orientation::O State;

public:
  RawOrientationsHandler(int chain_origin, RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& robot_state)
      : BasePointsHandler(chain_origin, model, robot_state, 3, 4)
  {
  }

  RawOrientationsHandler(std::string chain_origin, const mwoibn::robot_class::State& robot_state,
                         RigidBodyDynamics::Model& model)
      : BasePointsHandler(chain_origin, model, robot_state, 3, 4)
  {
  }

  RawOrientationsHandler(int chain_origin, RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& robot_state,
                         std::vector<Frame> points)
      : BasePointsHandler(chain_origin, model, robot_state, points, 3, 4)
  {
  }

  RawOrientationsHandler(std::string chain_origin,
                         RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& robot_state,
                         std::vector<Frame> points)
      : BasePointsHandler(chain_origin, model, robot_state, points, 3, 4)
  {
  }

  RawOrientationsHandler(RawOrientationsHandler&& other)
      : BasePointsHandler(other)
  {
  }

  RawOrientationsHandler(RawOrientationsHandler& other)
      : BasePointsHandler(other)
  {
  }

  template<typename Type, typename Vector>
  RawOrientationsHandler(Type chain_origin, RigidBodyDynamics::Model& model,
                         const mwoibn::robot_class::State& robot_state,
                         Vector reference_frames,
                         std::vector<State> states = {},
                         std::vector<std::string> names = {})
      : BasePointsHandler(chain_origin, model, robot_state, 3, 4)
  {
    _initFromVectors(reference_frames, states, names);
  }

  virtual ~RawOrientationsHandler() {}


  /** @brief returns Jacobian for a body as implemented in a base class
   *
   * @note whole Jacobian is overwritten
   */
  const mwoibn::Matrix& getPointJacobian(unsigned int id,
                                         bool update = false) const
  {
    //    std::cerr << "Orientation Handler " << std::endl;
    return _points.at(id)->getOrientationJacobian(update);
  }

  virtual unsigned int getPointJacobianRows(unsigned int id) const
  {
    return _points.at(id)->getOrientationJacobianRows();
  }

  using BasePointsHandler::addPoint;

  virtual unsigned int addPoint(State state, int body_id, std::string name = "")
  {
    _points.push_back(std::unique_ptr<Frame>(
        new Frame(Point::Current::Zero(3), body_id, _model, _robot_state, state, name)));

    _resize();

    return _points.size() - 1;
  }

  virtual void setPointStateFixed(unsigned int id, const State state)
  {
    _points.at(id)->setOrientationFixed(state);
  }

  const State& getPointStateFixed(unsigned int id)
  {
    return _points.at(id)->getOrientationFixed();
  }

  virtual void setPointStateWorld(unsigned int id, const State state,
                                  bool update = false)
  {
    _points.at(id)->setOrientationWorld(state, update);
  }

  virtual const State& getPointStateWorld(unsigned int id,
                                          bool update = false)
  {
    return _points.at(id)->getOrientationWorld(update);
  }

  virtual State getPointStateWorld(unsigned int id,
                                   bool update = false) const
  {
    return _points.at(id)->getOrientationWorld(update);
  }

  virtual State
  getPointStateReference(unsigned int id,
                         bool update = false) const
  {
    return _points.at(id)
        ->getOrientationReference(_reference, update);
  }

  virtual void setPointStateReference(unsigned int id, const State state,
                                      bool update = false)
  {
    _points.at(id)->setOrientationReference(state, _reference, update);
  }


  virtual void setFullStateReference(const mwoibn::VectorN state,
                                     bool update = false)
  {
    int k = 0;
    for (int i = 0; i < size(); i++)
    {
      mwoibn::Quaternion::fromVector(_state, state, k);
      setPointStateReference(i, _state, update);
      k += getStateSize();
      update = false;
    }
  }
  virtual void setFullStateWorld(const mwoibn::VectorN state,
                                 bool update = false)
  {
    int k = 0;
    for (int i = 0; i < size(); i++)
    {
      mwoibn::Quaternion::fromVector(_state, state, k);
      setPointStateWorld(i, _state, update);
      k += getStateSize();
      update = false;
    }
  }

  virtual void setFullStateFixed(const mwoibn::VectorN state)
  {
    int k = 0;
    for (int i = 0; i < size(); i++)
    {
      //mwoibn::VectorN temp_state = state.segment(k, _points.at(i)->size());
      mwoibn::Quaternion::fromVector(_state, state, k);
      setPointStateFixed(i, _state);
      k += getStateSize();
    }
  }

  virtual const mwoibn::VectorN& getFullStateWorld(bool update = false)
  {
    for (int i = 0; i < _points.size(); i++)
    {
      mwoibn::Quaternion::toVector(getPointStateWorld(i, update), _fullState, getStateSize() * i);
      update = false;
    }
    return _fullState;
  }

  mwoibn::VectorN getFullStateWorld(bool update = false) const
  {
    mwoibn::VectorN state;
    state.setZero(_fullState.size());

    for (int i = 0; i < _points.size(); i++)
    {
      mwoibn::Quaternion::toVector(getPointStateWorld(i, update), state, getStateSize() * i);
      update = false;
    }
    return state;
  }

  virtual const mwoibn::VectorN& getFullStateFixed()
  {

    for (int i = 0; i < _points.size(); i++)
      mwoibn::Quaternion::toVector(
          getPointStateFixed(i),
          _fullState, getStateSize() * i);

    return _fullState;
  }

  virtual const mwoibn::VectorN& getFullStateReference(bool update = false)
  {

//    mwoibn::VectorN states(getStateSize() * _points.size());
    for (int i = 0; i < _points.size(); i++)
    {
      mwoibn::Quaternion::toVector(getPointStateReference(i, update), _fullState, getStateSize() * i);

      update = false;
    }
    return _fullState;
  }
};

} // namespace package
} // namespace library

#endif // RAW_ORIENTATIONS_HANLDER_H
