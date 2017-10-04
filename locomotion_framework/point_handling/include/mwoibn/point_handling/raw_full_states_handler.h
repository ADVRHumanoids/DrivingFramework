#ifndef POINT_HANDLING_RAW_FULL_STATES_HANDLER_H
#define POINT_HANDLING_RAW_FULL_STATES_HANDLER_H

#include "mwoibn/point_handling/base_points_handler.h"
#include "mwoibn/point_handling/point.h"

namespace mwoibn
{

namespace point_handling
{

class RawFullStatesHandler : public BasePointsHandler<mwoibn::Vector7>
{
  typedef mwoibn::Vector7 State;

public:

  RawFullStatesHandler(int chain_origin, RigidBodyDynamics::Model& model)
      : BasePointsHandler(chain_origin, model, 6, 7)
  {  }

  RawFullStatesHandler(std::string chain_origin, RigidBodyDynamics::Model& model)
      : BasePointsHandler(chain_origin, model, 6, 7)
  {  }


  RawFullStatesHandler(int chain_origin, RigidBodyDynamics::Model& model,
                    std::vector<Point> points)
      : BasePointsHandler(chain_origin, model, points, 6, 7)
  {  }

  RawFullStatesHandler(std::string chain_origin, RigidBodyDynamics::Model& model,
                    std::vector<Point> points)
      : BasePointsHandler(chain_origin, model, points, 6, 7)
  {  }

  RawFullStatesHandler(RawFullStatesHandler&& other) : BasePointsHandler(other)
  {
  }

  RawFullStatesHandler(RawFullStatesHandler& other) : BasePointsHandler(other)
  {
  }

  RawFullStatesHandler(int chain_origin, RigidBodyDynamics::Model& model,
                       std::vector<int> reference_frames,
                       std::vector<State> states = {},
                       std::vector<std::string> names = {})
      : BasePointsHandler(chain_origin, model, 6, 7)
  {
    _initFromVectors<int>(reference_frames, states, names);
  }

  RawFullStatesHandler(std::string chain_origin,
                       RigidBodyDynamics::Model& model,
                       std::vector<std::string> reference_frames,
                       std::vector<State> states = {},
                       std::vector<std::string> names = {})
      : BasePointsHandler(chain_origin, model, 6, 7)
  {
    _initFromVectors<std::string>(reference_frames, states, names);
  }

  virtual ~RawFullStatesHandler() {}

  /** @brief returns Jacobian for a body as implemented in a base class
   *
   * @note whole Jacobian is overwritten
   */
  const mwoibn::Matrix& getPointJacobian(unsigned int id, const mwoibn::VectorN& joint_states,
                                  bool update = false) const
  {
    return _points.at(id)->getFullJacobian(joint_states, update);
  }

  unsigned int getPointJacobianRows(unsigned int id) const
  {
    return _points.at(id)->getFullJacobianRows();
  }

  using BasePointsHandler::addPoint;

  virtual unsigned int addPoint(State state, int body_id, std::string name = "")
  {
    _points.push_back(std::unique_ptr<Point>(new Point(
        Point::Position(state[0], state[1], state[2]), body_id, _model,
        Point::Orientation(state[3], state[4], state[5], state[6]), name)));

    _reducedJacobian.setZero(_points.size()*_jacobian_row, _chain.size());
    _fullJacobian.setZero(_points.size()*_jacobian_row, _model.dof_count);

    return _points.size() - 1;
  }

  virtual unsigned int addPoint(State state, std::string body_name,
                                std::string name = "")
  {

    _points.push_back(std::unique_ptr<Point>(new Point(
        Point::Position(state[0], state[1], state[2]), body_name, _model,
        Point::Orientation(state[3], state[4], state[5], state[6]), name)));

    _reducedJacobian.setZero(_points.size()*_jacobian_row, _chain.size());
    _fullJacobian.setZero(_points.size()*_jacobian_row, _model.dof_count);

    return _points.size() - 1;
  }

  virtual void setPointStateFixed(unsigned int id, const State state)
  {
    _points.at(id)->setFullStateFixed(state);
  }

  const State& getPointStateFixed(unsigned int id)
  {
    return _points.at(id)->getFullStateFixed();
  }

  virtual void setPointStateWorld(unsigned int id, const State state,
                                  const mwoibn::VectorN& joint_states,
                                  bool update = false)
  {
    _points.at(id)->setFullStateWorld(state, joint_states, update);
  }

  virtual const State& getPointStateWorld(unsigned int id,
                                          const mwoibn::VectorN& joint_states,
                                          bool update = false)
  {
    return _points.at(id)->getFullStateWorld(joint_states, update);
  }

  virtual State getPointStateWorld(unsigned int id,
                                   const mwoibn::VectorN& joint_states,
                                   bool update = false) const{

    return _points.at(id)->getFullStateWorld(joint_states, update);

  }


  virtual const State& getPointStateReference(unsigned int id,
                                       const mwoibn::VectorN& joint_states,
                                       bool update = false)
  {
    return _points.at(id)->getFullStateReference(_reference, joint_states, update);
  }

  virtual void setPointStateReference(unsigned int id, const State state,
                                      const mwoibn::VectorN& joint_states,
                                      bool update = false)
  {
    _points.at(id)->setFullStateReference(state, _reference, joint_states, update);
   }

  virtual void setFullStateReference(const mwoibn::VectorN state,
                                     const mwoibn::VectorN& joint_states,
                                     bool update = false)
  {
    int k = 0;
    for (int i = 0; i < size(); i++)
    {
      _state.noalias() = state.segment(k, _points.at(i)->size());
      setPointStateReference(i, _state, joint_states, update);
      k += getStateSize();
      update = false;
    }
  }
  virtual void setFullStateWorld(const mwoibn::VectorN state,
                                 const mwoibn::VectorN& joint_states,
                                 bool update = false)
  {
    int k = 0;
    for (int i = 0; i < size(); i++)
    {
//      State temp;
      _state.noalias() = state.segment(
          k, _points.at(i)
                 ->size()); // here the proper size has to ensured by a user

      setPointStateWorld(i, _state, joint_states, update);
      k += getStateSize();
      update = false;
    }
  }
  virtual void setFullStateFixed(const mwoibn::VectorN state)
  {
    int k = 0;
    for (int i = 0; i < size(); i++)
    {
      _state.noalias() = state.segment(k, getStateSize());
      setPointStateFixed(i, _state);
      k += getStateSize();
    }
  }

  virtual const mwoibn::VectorN& getFullStateWorld(const mwoibn::VectorN& joint_states,
                                            bool update = false)
  {

//    mwoibn::VectorN states(getStateSize() * _points.size());
    for (int i = 0; i < _points.size(); i++)
    {
      _fullState.segment(getStateSize() * i, getStateSize()) =
          getPointStateWorld(i, joint_states, update);
      update = false;
    }
    return _fullState;
  }

  /** @brief Returns a RBDL vector of States for all points. States are returned
   *in the points own frames. State \f$ x \f$ is in the same order as in
   *equation
   *\f$ \dot x = J \dot q \f$
   *
   */
  virtual const mwoibn::VectorN& getFullStateFixed()
  {

    for (int i = 0; i < _points.size(); i++)
      _fullState.segment(getStateSize() * i, getStateSize()) =
          getPointStateFixed(i);

    return _fullState;
  }

  /** @brief Returns a RBDL vector of States for all points. States are returned
   *in a PointsHandler reference frame. State \f$ x \f$ is in the same order as
   *in equation
   *\f$ \dot x = J \dot q \f$
   *
   */
  virtual const mwoibn::VectorN& getFullStateReference(const mwoibn::VectorN& joint_states,
                                                bool update = false)
  {

    for (int i = 0; i < _points.size(); i++)
    {
      _fullState.segment(getStateSize() * i, getStateSize()) =
          getPointStateReference(i, joint_states, update);
      update = false;
    }
    return _fullState;
  }


};

} // namespace package
} // namespace library

#endif // RAW_FULL_POINTS_HANDLER_H
