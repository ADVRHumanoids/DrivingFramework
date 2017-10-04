#ifndef POINT_HANDLING_STATE_POINTS_HANDLER_H
#define POINT_HANDLING_STATE_POINTS_HANDLER_H

#include "mwoibn/point_handling/raw_positions_handler.h"
#include "mwoibn/point_handling/raw_orientations_handler.h"
#include "mwoibn/point_handling/raw_full_states_handler.h"

namespace mwoibn
{

namespace point_handling
{
//typedef mwoibn::Vector3 State;

template <typename rawHandler, typename State> class StatePointsHandler : public rawHandler
{
public:
  StatePointsHandler(int chain_origin, RigidBodyDynamics::Model& model,
                     const mwoibn::VectorN& positions)
      : rawHandler(chain_origin, model), _positions(positions)
  {
  }

  StatePointsHandler(std::string chain_origin, RigidBodyDynamics::Model& model,
                     const mwoibn::VectorN& positions)
      : rawHandler(chain_origin, model), _positions(positions)
  {
  }

  StatePointsHandler(int chain_origin, RigidBodyDynamics::Model& model,
                     const mwoibn::VectorN& positions,
                     std::vector<Point> points)
      : rawHandler(chain_origin, model, points), _positions(positions)
  {
  }

  StatePointsHandler(std::string chain_origin, RigidBodyDynamics::Model& model,
                     const mwoibn::VectorN& positions,
                     std::vector<Point> points)
      : rawHandler(chain_origin, model, points), _positions(positions)
  {
  }

  StatePointsHandler(int chain_origin, RigidBodyDynamics::Model& model,
                     const mwoibn::VectorN& positions,
                     std::vector<int> reference_frames,
                     std::vector<State> states = {},
                     std::vector<std::string> names = {})
      : rawHandler(chain_origin, model, reference_frames, states, names),
        _positions(positions)
  {
  }

  StatePointsHandler(std::string chain_origin, RigidBodyDynamics::Model& model,
                     const mwoibn::VectorN& positions,
                     std::vector<std::string> reference_frames,
                     std::vector<State> states = {},
                     std::vector<std::string> names = {})
      : rawHandler(chain_origin, model, reference_frames, states, names),
        _positions(positions)
  {
  }

  StatePointsHandler(StatePointsHandler&& other)
      : rawHandler(std::move(other)), _positions(other._positions)
  {
  }
  StatePointsHandler(StatePointsHandler& other)
      : rawHandler(other), _positions(other._positions)
  {
  }
  virtual ~StatePointsHandler() {}

  virtual const mwoibn::Matrix& getFullJacobian()
  {
    return rawHandler::getFullJacobian(_positions);
  }

  virtual std::vector<mwoibn::Matrix>
  getFullJacobians()
  {

    return rawHandler::getFullJacobians(_positions, false);
  }

  virtual const mwoibn::Matrix& getFullPointJacobian(
      unsigned int id)
  {
    return rawHandler::getFullPointJacobian(id, _positions, false);
  }

  virtual const mwoibn::VectorN& getFullStateReference()
  {
    return rawHandler::getFullStateReference(_positions, false);
  }

  virtual std::vector<State> getFullStatesReference()
  {
    return rawHandler::getFullStatesReference(_positions, false);
  }

  virtual std::vector<State> getFullStatesWorld()
  {
    return rawHandler::getFullStatesWorld(_positions, false);
  }

  virtual const mwoibn::VectorN& getFullStateWorld()
  {
    return rawHandler::getFullStateWorld(_positions, false);
  }

  virtual const mwoibn::Matrix&
  getPointJacobian(unsigned int id) const
  {
    return rawHandler::getPointJacobian(id, _positions, false);
  }

  virtual const State& getPointStateReference(unsigned int id)
  {
    return rawHandler::getPointStateReference(id, _positions, false);
  }

  virtual const State& getPointStateWorld(unsigned int id)
  {
    return rawHandler::getPointStateWorld(id, _positions, false);
  }

  virtual State getPointStateWorld(unsigned int id) const
  {
    return rawHandler::getPointStateWorld(id, _positions, false);
  }

  virtual const mwoibn::Matrix& getReducedJacobian()
  {
    return rawHandler::getReducedJacobian(_positions, false);
  }

  virtual std::vector<mwoibn::Matrix>
  getReducedJacobians()
  {
    return rawHandler::getReducedJacobians(_positions, false);
  }

  virtual mwoibn::Matrix
  getReducedPointJacobian(unsigned int id)
  {
    return rawHandler::getReducedPointJacobian(id, _positions, false);
  }

  virtual void setPointStateReference(unsigned int id, const State state)
  {
    rawHandler::setPointStateReference(id, state, _positions, false);
  }

  virtual void setPointStateWorld(unsigned int id, State state)
  {
    rawHandler::setPointStateWorld(id, state, _positions, false);
  }

  virtual void setFullStateWorld(const mwoibn::VectorN& state)
  {
    rawHandler::setFullStateWorld(state, _positions, false);
  }

  virtual void setFullStateReference(const mwoibn::VectorN state)
  {
    rawHandler::setFullStateReference(state, _positions, false);
  }




protected:
  const mwoibn::VectorN& _positions;
};

} // namespace package
} // namespace library
#endif // ROBOT_POINTS_HANDLER_H
