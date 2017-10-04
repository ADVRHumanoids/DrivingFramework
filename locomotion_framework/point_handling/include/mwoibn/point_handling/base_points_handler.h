#ifndef POINT_HANDLING_BASE_POINTS_HANDLER_H
#define POINT_HANDLING_BASE_POINTS_HANDLER_H

#include "mwoibn/point_handling/point_handling.h"
#include "mwoibn/point_handling/point.h"
#include <memory>
#include <vector>

namespace mwoibn
{
namespace point_handling
{

template <typename State> class BasePointsHandler
{

public:
  BasePointsHandler(int chain_origin, RigidBodyDynamics::Model& model, int jacobian_row, int state_size)
      : _reference(chain_origin), _model(model), _jacobian_row(jacobian_row), _state_size(state_size)
  {
    _fullPointJacobian.setZero(_jacobian_row, _model.dof_count);
  }

  BasePointsHandler(std::string chain_origin, RigidBodyDynamics::Model& model, int jacobian_row, int state_size)
      : _reference(_checkBody(chain_origin, model)), _model(model), _jacobian_row(jacobian_row), _state_size(state_size)
  {
    _fullPointJacobian.setZero(_jacobian_row, _model.dof_count);
  }

  virtual ~BasePointsHandler() {}

  BasePointsHandler(int chain_origin, RigidBodyDynamics::Model& model,
                    std::vector<Point> points, int jacobian_row, int state_size)
      : _reference(chain_origin), _model(model), _jacobian_row(jacobian_row), _state_size(state_size)  {
    _fullPointJacobian.setZero(_jacobian_row, _model.dof_count);

    for (auto& point : points)
      addPoint(point);

    computeChain();
  }

  BasePointsHandler(std::string chain_origin, RigidBodyDynamics::Model& model,
                    std::vector<Point> points, int jacobian_row, int state_size)
      : _reference(_checkBody(chain_origin, model)), _model(model), _jacobian_row(jacobian_row), _state_size(state_size)  {
    _fullPointJacobian.setZero(_jacobian_row, _model.dof_count);

    for (auto& point : points)
      addPoint(point);

    computeChain();
  }

  BasePointsHandler(const BasePointsHandler& other)
      : _reference(other._reference), _chain(other._chain), _model(other._model), _jacobian_row(other._jacobian_row), _state_size(other._state_size)
  {
    _fullPointJacobian.setZero(_jacobian_row, _model.dof_count);

    _reducedPointJacobian.setZero(_jacobian_row, _chain.size());

    for (auto& point : other._points)
      addPoint(*point);
  }

  BasePointsHandler(const BasePointsHandler&& other)
      : _reference(other._reference), _chain(other._chain), _model(other._model), _jacobian_row(other._jacobian_row), _state_size(other._state_size)
  {
    _fullPointJacobian.setZero(_jacobian_row, _model.dof_count);
    _reducedPointJacobian.setZero(_jacobian_row, _chain.size());

    for (auto& point : other._points)
      addPoint(*point);
  }

  /** @name Unique Point
   *
   * These methods provided access to analogus methods of a Point class, and
   *possiblity to add/remove points from PointHandler
   */
  ///@{

  /** @brief add new point to the PointsHandler
   *
   * This function adds new point to the PointsHandler at the end of a stack
   *
   * @warning for preformance preposes this function does not call
   *computeChain() automatically. If new fixed frames are added to the
   * PointsHandler computeChain has to be called explicitly by the user
   *
   * @return id of a new point
   *
   */
  unsigned int addPoint(Point point)
  {
    _points.push_back(std::unique_ptr<Point>(new Point(point)));

    _reducedJacobian.setZero(_points.size() * _jacobian_row, _chain.size());
    _fullJacobian.setZero(_points.size() * _jacobian_row, _model.dof_count);
    _fullState.setZero(getStateSize() * _points.size());

    return _points.size() - 1;
  }

  /** @brief add new point to the PointsHandler
   *
   * This function adds new point to the PointsHandler at the end of a stack
   *
   * @warning for preformance preposes this function does not call
   *computeChain() automatically. If new fixed frames are added to the
   * PointsHandler computeChain has to be called explicitly by the user
   *
   * @return id of a new point
   *
   */
  unsigned int addPoint(int body_id, std::string name = "")
  {
    _points.push_back(std::unique_ptr<Point>(new Point(body_id, _model, name)));
    _reducedJacobian.setZero(_points.size() * _jacobian_row, _chain.size());
    _fullJacobian.setZero(_points.size() * _jacobian_row, _model.dof_count);
    _fullState.setZero(getStateSize() * _points.size());

    return _points.size() - 1;
  }

  /** @brief add new point to the PointsHandler
   *
   * This function adds new point to the PointsHandler at the end of a stack
   *
   * @warning for preformance preposes this function does not call
   *computeChain() automatically. If new fixed frames are added to the
   * PointsHandler computeChain has to be called explicitly by the user
   *
   * @return id of a new point
   *
   */
  unsigned int addPoint(std::string body_name, std::string name = "")
  {
    int body_id;
    try
    {
      body_id = _checkBody(body_name);
    }
    catch (const std::invalid_argument& e)
    {
      throw;
    }

    return addPoint(body_id, name);
  }

  /** @brief add new point to the PointsHandler
   *
   * This function adds new point to the PointsHandler at the end of a stack
   *
   * @warning for preformance preposes this function does not call
   *computeChain() automatically. If new fixed frames are added to the
   * PointsHandler computeChain has to be called explicitly by the user
   *
   * @return id of a new point
   *
   */
  virtual unsigned int addPoint(State state, int body_id,
                                std::string name = "") = 0;

  /** @brief add new point to the PointsHandler
   *
   * This function adds new point to the PointsHandler at the end of a stack
   *
   * @warning for preformance preposes this function does not call
   *computeChain() automatically. If new fixed frames are added to the
   * PointsHandler  has to be called explicitly by the user
   *
   * @return id of a new point
   *
   */
  virtual unsigned int addPoint(State state, std::string body_name,
                                std::string name = "")
  {
    int body_id;
    try
    {
      body_id = _checkBody(body_name);
    }
    catch (const std::invalid_argument& e)
    {
      throw;
    }

    return addPoint(state, body_id, name);
  }

  /** @brief return number of point defined in a points handler */
  int size() const { return _points.size(); }

  /** @brief set new tracked point for a Point id giving data in a point fixed
   *frame
   *
   * @see Point::setStateFixed
   */
  virtual void setPointStateFixed(unsigned int id, const State state) = 0;

  /** @brief get state of a point id in its fixed frame
   *
   * @see Point::getStateFixed
   */
  virtual const State& getPointStateFixed(unsigned int id) = 0;

  /** @brief set new tracked point for a Point id giving data in a world frame
   *
   * @see Point::setStateWorld
   */
  virtual void setPointStateWorld(unsigned int id, const State state,
                                  const mwoibn::VectorN& joint_states,
                                  bool update = false) = 0;

  /** @brief get State of a Point id in a world frame
   *
   * @see Point::getStateWorld
   */
  virtual const State& getPointStateWorld(unsigned int id,
                                          const mwoibn::VectorN& joint_states,
                                          bool update = false) = 0;

  virtual State getPointStateWorld(unsigned int id,
                                   const mwoibn::VectorN& joint_states,
                                   bool update = false) const = 0;
  /** @brief get State in a PointHandler reference frame
   *
   * @see Point::getStateReference
   */
  virtual const State&
  getPointStateReference(unsigned int id, const mwoibn::VectorN& joint_states,
                         bool update = false) = 0;

  /** @brief set new tracked point for a Pint id giving data in a PointHandler
   *reference frame
   *
   * @see Point::setStateReference
   */
  virtual void setPointStateReference(unsigned int id, const State state,
                                      const mwoibn::VectorN& joint_states,
                                      bool update = false) = 0;

  /** @brief set new tracked point for a Pint id giving data in a PointHandler
   *reference frame
   *
   * @see Point::setStateReference
   */
  virtual void setFullStateReference(const mwoibn::VectorN state,
                                     const mwoibn::VectorN& joint_states,
                                     bool update = false) = 0;

  virtual void setFullStateWorld(const mwoibn::VectorN state,
                                 const mwoibn::VectorN& joint_states,
                                 bool update = false) = 0;

  virtual void setFullStateFixed(const mwoibn::VectorN state) = 0;
  /** @brief returns Point name */
  std::string getPointName(unsigned int id) const
  {
    return _points.at(id)->getName();
  }

  /** @brief returns first point with given name */

  unsigned int getPointId(std::string name) const
  {
    auto it = std::find_if(_points.begin(), _points.end(),
                           [name](const std::unique_ptr<Point>& point)
                           {
                             return point->getName() == name;
                           });

    if (it == _points.end())
      return std::numeric_limits<unsigned int>::max();
    else
      return std::distance(_points.begin(), it);
  }

  /** @brief returns vector of all points with given name */
  std::vector<unsigned int> getPointIds(std::string name) const
  {

    std::vector<unsigned int> ids;

    for (int i = 0; i < _points.size(); i++)
    {
      if (_points.at(i)->getName() == name)
        ids.push_back(i);
    }
    return ids;
  }

  /** @brief returns Jacobian for a body as implemented in a base class
   *
   * @note whole Jacobian is overwritten
   */
  virtual const mwoibn::Matrix&
  getPointJacobian(unsigned int id, const mwoibn::VectorN& joint_states,
                   bool update = false) const = 0;

  /** @brief returns reduced Jacobian in which only dofs in a chain are
   *provided in a world frame
   *
   * @note The dofs are returned in a chain order
   *
   * @note Whole Jacobian is overwitten
   *
   * @see getReducedJacobian(), getReducedJacobians()
   */
  virtual const mwoibn::Matrix&
  getReducedPointJacobian(unsigned int id, const mwoibn::VectorN& joint_states,
                          bool update = false)
  {
    _fullPointJacobian.noalias() = getPointJacobian(id, joint_states, update);

    for (int j = 0; j < _chain.size(); j++)
    {
      _reducedPointJacobian.col(j) = _fullPointJacobian.col(_chain.at(j));
    }

    return _reducedPointJacobian;
  }

  /** @brief returns reduced Jacobian with a full state, the dofs not in a chain
   *are zero
   *
   * @warning only non-zero columns are evaluated, size of a matrix send to the
   *method has to be correct. See also getFullPointJacobianRows(),
   *getFullPointJacobianCols()
   *
   * @see getFullJacobian(), getFullJacobians()
   *
   */
  virtual const mwoibn::Matrix&
  getFullPointJacobian(unsigned int id, const mwoibn::VectorN& joint_states,
                       bool update = false)
  {

    _fullPointJacobian.noalias() = getPointJacobian(id, joint_states, update);

    for (auto i : _empty)
    {
      _fullPointJacobian.col(i).setZero();
    }

    return _fullPointJacobian;
  }


  virtual unsigned int getPointJacobianRows(unsigned int id) const = 0;

  /** @brief returns number of cols expected in a Matrix sent to the
   *getPointFullJacobian() method for a point id
   *
   * @see getFullPointJacobianRows(), getFullJacobianRows(),
   *getFullJacobianCols()
   */
  unsigned int getFullPointJacobianCols(unsigned int id) const
  {
    return _model.dof_count;
  }

  /** @brief remove given point from PointHandler
   *
   * @warning for preformance preposes this function does not call
   *() automatically. If new fixed frames are removed to the
   * PointsHandler  has to be called explicitly by the user
   *
   */
  bool removePoint(unsigned int id)
  {
    if (id >= _points.size())
      return false;

    _points.erase(_points.begin() + id);
    _reducedJacobian.setZero(_points.size() * _jacobian_row, _chain.size());
    _fullJacobian.setZero(_points.size() * _jacobian_row, _model.dof_count);

    return true;
  }
  ///@}

  /** @name General
   *
   */
  ///@{

  //! Allows to recompute from the reduced Jacobian to full RBDL model reference
  /**
   * reduced Jacobian considers only the dofs from the reference frame to the
   * end-effector
   * @note This is mostly desigend as internal function, but may be useful if
   * implemented algorithm works on the limited part of the system
   */
  void mapIKtoRBDL(mwoibn::Matrix& J_full, mwoibn::Matrix J) const
  {

    for (int j = 0; j < J.cols(); j++)
    {
      J_full.middleCols(_chain.at(j), 1) = J.middleCols(j, 1);
    }
  }

  //! Allows to recompute from the reduced Jacobian to full RBDL model reference
  /**
   * reduced Jacobian considers only the dofs from the reference frame to the
   * end-effector
   * @note This is mostly desigend as internal function, but may be useful if
   * implemented algorithm works on the limited part of the system
   */
  void mapIKtoRBDL(mwoibn::VectorN& q_full, mwoibn::VectorN q) const
  {

    for (int j = 0; j < q.size(); j++)
    {
      q_full[_chain.at(j)] = q[j];
    }
  }

  /** @brief compute the chain from the PointsHandler origin to the fixed frames
   *of each point
   *
   * @warning for preformance preposes this function is not called when points
   *are added/removed from a PointsHandler. If new fixed frames are added to the
   *PointsHandler is has to be called explicitly by the user
   *
   * @note the function is called automatically at the end of initialization
   *procces
   */
  bool computeChain()
  {
    int final_id = (_reference > _model.dof_count)
                       ? _model.GetParentBodyId(_reference)
                       : _reference;
    int current_id = 0;
    bool success;

    std::vector<RigidBodyDynamics::Joint> mJoints = _model.mJoints;

    for (int i = 0; i < _points.size(); i++)
    {
      success = false;
      current_id = _points.at(i)->getBodyId();

      while (!success)
      {

        if (current_id == final_id)
        {
          success = true;
        }
        else if (current_id > _model.dof_count)
        {
          current_id = _model.GetParentBodyId(current_id);
        }
        else if (current_id == 0)
        {
          throw(std::domain_error(
              "could not find the path from body " +
              _model.GetBodyName(_points.at(i)->getBodyId()) +
              " to defined origin " + _model.GetBodyName(_reference)));
          return success;
        }
        else
        {
          for (int temp = 0; temp < mJoints[current_id].mDoFCount; temp++)
            _chain.push_back(mJoints[current_id].q_index + temp);

          current_id = _model.GetParentBodyId(current_id);
        }
      }
    }
    sort(_chain.begin(), _chain.end());
    _chain.erase(unique(_chain.begin(), _chain.end()), _chain.end());

    _reducedJacobian.setZero(_points.size() * _jacobian_row, _chain.size());
    _reducedPointJacobian.setZero(_jacobian_row, _chain.size());

    std::vector<int> full_list(_model.dof_count);
    std::iota(full_list.begin(), full_list.end(), 0);
    std::remove_copy_if(_chain.begin(), _chain.end(),
                        std::back_inserter(_empty), [&full_list](const int& arg)
                        {
                          return (std::find(full_list.begin(), full_list.end(),
                                            arg) != full_list.end());
                        });

    for (auto i : _empty)
      std::cout << "empty " << i << std::endl;

    return success;
  }

  const Point& point(int i) { return (*_points.at(i)); }
  /** @brief returns selector for a given point_handler **/
  virtual std::vector<int> getChain() { return _chain; }

  /** @brief returns an RDBL id of a PointsHandler reference frame (base) */
  unsigned int getOriginId() const { return _reference; }

  /** @brief returns a human-readable name of a PointsHandler reference frame
   * (base) */
  std::string getOriginName() const { return _model.GetBodyName(_reference); }
  ///@}

  /** @name Jacobian
   *
   */
  ///@{

  /** @brief returnes common Jacobian for all the points, considering only robot
   * DOFs present in the PointsHandler chain
   *
   * @see getReducedPointJacobian(), getReducedJacobians()
   */
  virtual const mwoibn::Matrix& getReducedJacobian(const mwoibn::VectorN& joint_states,
                                            bool update = false)
  {
    for (int i = 0; i < _points.size(); i++)
    {
      _reducedJacobian.middleRows(i * getPointJacobianRows(i), getPointJacobianRows(i)) =
          getReducedPointJacobian(i, joint_states, update);
      update = false;
    }

    return _reducedJacobian;
  }

  /** @brief returnes one Jacobian for all the points, considering all robot
   * DOFs, evaluates only columns related to the DOFs present in a PointsHandler
   *chain
   *
   * @see getFullPointJacobian(), getFullJacobians()
   */
  virtual const mwoibn::Matrix& getFullJacobian(const mwoibn::VectorN& joint_states,
                               bool update = false)
  {
    for (int i = 0; i < _points.size(); i++)
    {
      _fullJacobian.middleRows(i * getPointJacobianRows(i), getPointJacobianRows(i)) =
          getFullPointJacobian(i, joint_states, update);
      update = false;
    }
    return _fullJacobian;
  }

  /** @brief returnes vector of reduced Jacobians for all the points,
   *considering only robot
   * DOFs present in the PointsHandler chain
   *
   * @see getReducedPointJacobian(), getReducedJacobian()
   */
  virtual std::vector<mwoibn::Matrix>
  getReducedJacobians(mwoibn::VectorN joint_states, bool update = false)
  {

    std::vector<mwoibn::Matrix> Js;

    for (int i = 0; i < _points.size(); i++)
    {
      Js.push_back(getReducedPointJacobian(i, joint_states, update));
      update = false;
    }

    return Js;
  }

  /** @brief returnes vector of Jacobians for all the points, considering all
   *robot
   * DOFs, evaluates only columns related to the DOFs present in a PointsHandler
   *chain,
   *  the columns related to the dofs not present in a chain are equal zero.
   *
   * @see getFullPointJacobian(), getFullJacobian()
   */
  virtual std::vector<mwoibn::Matrix>
  getFullJacobians(mwoibn::VectorN joint_states, bool update = false)
  {
    std::vector<mwoibn::Matrix> Js;
    for (int i = 0; i < _points.size(); i++)
    {
      Js.push_back(getFullPointJacobian(i, joint_states, update));
      update = false;
    }

    return Js;
  }

  /** @brief returns number of rows expected in a Matrix sent to the
   *getFullJacobian() method
   *
   * @see  getFullJacobianCols(), getFullPointJacobianRows(),
   *getFullPointJacobianCols(),
   */
  unsigned int getFullJacobianRows() const
  {
    return _points.size() * _jacobian_row;
  }
  /** @brief returns number of cols expected in a Matrix sent to the
   *getFullJacobian() method
   *
   * @see  getFullJacobianRows(), getFullPointJacobianRows(),
   *getFullPointJacobianCols(),
   */
  unsigned int getFullJacobianCols() const { return _model.dof_count; }

  ///@}

  /** @name State
   *
   */
  ///@{

  /** @brief Returns a RBDL vector of States for all points. States are returned
   *in the world frame. State \f$ x \f$ is in the same order as in equation
   *\f$ \dot x = J \dot q \f$
   *
   */
  virtual const mwoibn::VectorN& getFullStateWorld(const mwoibn::VectorN& joint_states,
                                            bool update = false) = 0;

  /** @brief Returns a RBDL vector of States for all points. States are returned
   *in the points own frames. State \f$ x \f$ is in the same order as in
   *equation
   *\f$ \dot x = J \dot q \f$
   *
   */
  virtual const mwoibn::VectorN& getFullStateFixed() = 0;

  /** @brief Returns a RBDL vector of States for all points. States are returned
   *in a PointsHandler reference frame. State \f$ x \f$ is in the same order as
   *in equation
   *\f$ \dot x = J \dot q \f$
   *
   */
  virtual const mwoibn::VectorN& getFullStateReference(const mwoibn::VectorN& joint_states,
                                                bool update = false) = 0;

  /** @brief Returns a std::vector of states for all points. States are returned
   *in the world frame.
   *
   */
  virtual std::vector<State> getFullStatesWorld(mwoibn::VectorN joint_states,
                                                bool update = false)
  {

    std::vector<State> states;
    for (int i = 0; i < _points.size(); i++)
    {
      states.push_back(getPointStateWorld(i, joint_states, update));
      update = false;
    }
    return states;
  }

  /** @brief Returns a std::vector of states for all points. States are returned
   *in points own frames.
   *
   */
  virtual std::vector<State> getFullStatesFixed()
  {

    std::vector<State> states;
    for (int i = 0; i < _points.size(); i++)
      states.push_back(getPointStateFixed(i));

    return states;
  }
  /** @brief Returns a std::vector of states for all points. States are returned
   *in a PointsHandler reference frame.
   *
   */
  virtual std::vector<State>
  getFullStatesReference(mwoibn::VectorN joint_states, bool update = false)
  {

    std::vector<State> states;
    for (int i = 0; i < _points.size(); i++)
    {
      states.push_back(getPointStateReference(i, joint_states, update));
      update = false;
    }
    return states;
  }

  virtual unsigned int getStateSize() const {return _state_size;}
  ///@}

protected:
  /** @brief Keeps all links in a chain from point reference frame to a
   * user-defined
   * origin of a chain
   */
  int _jacobian_row = 0, _state_size = 0;
  std::vector<int> _chain, _empty;
  mwoibn::Matrix _fullPointJacobian, _reducedPointJacobian, _fullJacobian,
      _reducedJacobian;
   mwoibn::VectorN _fullState;
  State _state;
  /** keeps all the possible data*/
  std::vector<std::unique_ptr<Point>> _points;

  RigidBodyDynamics::Model& _model;

  unsigned int _reference;

  unsigned int _checkBody(std::string body_name) const
  {
    return _checkBody(body_name, _model);
  }

  unsigned int _checkBody(std::string body_name,
                          RigidBodyDynamics::Model model) const
  {

    unsigned body_id = model.GetBodyId(body_name.c_str());

    if (body_id == std::numeric_limits<unsigned int>::max())
    {

      throw(std::invalid_argument("unknown body, " + body_name +
                                  " couldn't find it in a RBDL model"));
    }
    return body_id;
  }

  template <typename Type>
  void _initFromVectors(std::vector<Type> reference_frames,
                        std::vector<State> states,
                        std::vector<std::string> names)
  { // chack if data have consistens sizes
    if (states.size() && states.size() != reference_frames.size())
      throw(std::invalid_argument("Couldn't initialize a PointHandler, wrong "
                                  "size of argument \"positions\""));
    if (names.size() && names.size() != reference_frames.size())
      throw(std::invalid_argument("Couldn't initialize a PointHandler, wrong "
                                  "size of argument \"names\""));

    if (!states.size() & !names.size())
    {
      for (auto& frame : reference_frames)
        addPoint(frame);
    }
    else if (!names.size())
    {
      for (int i = 0; i < reference_frames.size(); i++)
        addPoint(states.at(i), reference_frames.at(i));
    }
    else if (!states.size())
      for (int i = 0; i < reference_frames.size(); i++)
        addPoint(reference_frames.at(i), names.at(i));
    else
      for (int i = 0; i < reference_frames.size(); i++)
        addPoint(states.at(i), reference_frames.at(i), names.at(i));

    computeChain();
  }
};
} // namespace package
} // namespace library

#endif
