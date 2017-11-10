#ifndef POINT_HANDLING_RAW_POSITIONS_HANDLER_H
#define POINT_HANDLING_RAW_POSITIONS_HANDLER_H

#include "mwoibn/point_handling/base_points_handler.h"
#include "mwoibn/point_handling/point.h"

namespace mwoibn
{
namespace point_handling
{

class RawPositionsHandler : public BasePointsHandler<Point::Position>
{
  typedef Point::Position State;

public:
  RawPositionsHandler(int chain_origin, RigidBodyDynamics::Model& model)
      : BasePointsHandler(chain_origin, model, 3, 3)
  {
  }

  RawPositionsHandler(std::string chain_origin, RigidBodyDynamics::Model& model)
      : BasePointsHandler(chain_origin, model, 3, 3)
  {
  }

  RawPositionsHandler(int chain_origin, RigidBodyDynamics::Model& model,
                      std::vector<Point> points)
      : BasePointsHandler(chain_origin, model, points, 3, 3)
  {
  }

  RawPositionsHandler(std::string chain_origin, RigidBodyDynamics::Model& model,
                      std::vector<Point> points)
      : BasePointsHandler(chain_origin, model, points, 3, 3)
  {
  }

  RawPositionsHandler(RawPositionsHandler&& other) : BasePointsHandler(other) {}

  RawPositionsHandler(RawPositionsHandler& other) : BasePointsHandler(other) {}

  template <typename Type, typename Vector>
  RawPositionsHandler(Type chain_origin, RigidBodyDynamics::Model& model,
                      Vector reference_frames, std::vector<State> states = {},
                      std::vector<std::string> names = {})
      : BasePointsHandler(chain_origin, model, 3, 3)
  {
    _initFromVectors(reference_frames, states, names);
  }

  virtual ~RawPositionsHandler() {}

  /** @brief returns Jacobian for a body as implemented in a base class
   *
   * @note whole Jacobian is overwritten
   */
  const mwoibn::Matrix& getPointJacobian(unsigned int id,
                                         const mwoibn::VectorN& joint_states,
                                         bool update = false) const
  {
    //    std::cerr << "Poistions Handler " << std::endl;

    return _points.at(id)->getPositionJacobian(joint_states, update);
  }

  virtual unsigned int getPointJacobianRows(unsigned int id) const
  {
    return _points.at(id)->getPositionJacobianRows();
  }

  using BasePointsHandler::addPoint;

  virtual unsigned int addPoint(State state, int body_id, std::string name = "")
  {
    _points.push_back(std::unique_ptr<Point>(new Point(
        state, body_id, _model, Point::Orientation(0, 0, 0, 1), name)));

    _resize();

    return _points.size() - 1;
  }

  void setPointStateFixed(unsigned int id, const State state)
  {
    _points.at(id)->setPositionFixed(state);
  }

  const State& getPointStateFixed(unsigned int id)
  {
    return _points.at(id)->getPositionFixed();
  }

  virtual void setPointStateWorld(unsigned int id, const State state,
                                  const mwoibn::VectorN& joint_states,
                                  bool update = false)
  {
    _points.at(id)->setPositionWorld(state, joint_states, update);
  }

  virtual const State& getPointStateWorld(unsigned int id,
                                          const mwoibn::VectorN& joint_states,
                                          bool update = false)
  {
    return _points.at(id)->getPositionWorld(joint_states, update);
  }

  virtual State getPointStateWorld(unsigned int id,
                                   const mwoibn::VectorN& joint_states,
                                   bool update = false) const
  {
    return _points.at(id)->getPositionWorld(joint_states, update);
  }

  virtual const State&
  getPointStateReference(unsigned int id, const mwoibn::VectorN& joint_states,
                         bool update = false)
  {
    return _points.at(id)
        ->getPositionReference(_reference, joint_states, update);
  }

  virtual void setPointStateReference(unsigned int id, const State state,
                                      const mwoibn::VectorN& joint_states,
                                      bool update = false)
  {
    _points.at(id)
        ->setPositionReference(state, _reference, joint_states, update);
  }

  virtual void setFullStateReference(const mwoibn::VectorN state,
                                     const mwoibn::VectorN& joint_states,
                                     bool update = false)
  {
    int k = 0;
    for (int i = 0; i < size(); i++)
    {
      _state.noalias() = state.segment(
          k, _points.at(i)
                 ->size()); // here the proper size has to ensured by a user
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
      _state.noalias() = state.segment(k, _points.at(i)->size());

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
      //      State temp;
      _state.noalias() = state.segment(k, _points.at(i)->size());
      setPointStateFixed(i, state);
      k += getStateSize();
    }
  }

  virtual const mwoibn::VectorN&
  getFullStateWorld(const mwoibn::VectorN& joint_states, bool update = false)
  {
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
  virtual const mwoibn::VectorN&
  getFullStateReference(const mwoibn::VectorN& joint_states,
                        bool update = false)
  {

    //    mwoibn::VectorN states(getStateSize() * _points.size());
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

#endif
