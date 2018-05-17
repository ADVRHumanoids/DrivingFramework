#ifndef POINT_HANDLING_POINT_H
#define POINT_HANDLING_POINT_H

#include "mwoibn/point_handling/point_handling.h"

namespace mwoibn
{

namespace point_handling
{

class Point
{

public:
  typedef mwoibn::Vector3 Position;
  typedef mwoibn::Quaternion Orientation;
  typedef mwoibn::Matrix3 Rotation;

  Point(unsigned int body_id, RigidBodyDynamics::Model& model,
        std::string name = "")
      : _body_id(body_id), _name(name), _model(model)
  {
    _position << 0, 0, 0;
    _J.setZero(6, _model.dof_count);
    _J_part.setZero(3, _model.dof_count);
  }

  Point(std::string body_name, RigidBodyDynamics::Model& model,
        std::string name = "")
      : _name(name), _model(model), _body_id(_checkBody(body_name, model))
  {
    _position << 0, 0, 0;
    _J.setZero(6, _model.dof_count);
    _J_part.setZero(3, _model.dof_count);
  }

  Point(Point::Position position, unsigned int body_id,
        RigidBodyDynamics::Model& model,
        Point::Orientation orientation = Point::Orientation(0, 0, 0, 1),
        std::string name = "")
      : _body_id(body_id), _name(name), _model(model), _position(position),
        _orientation(orientation)
  {
    _J.setZero(6, _model.dof_count);
    _J_part.setZero(3, _model.dof_count);
  }

  Point(Point::Position position, std::string body_name,
        RigidBodyDynamics::Model& model,
        Point::Orientation orientation = Point::Orientation(0, 0, 0, 1),
        std::string name = "")
      : _name(name), _model(model), _body_id(_checkBody(body_name, model)),
        _orientation(orientation), _position(position)
  {
    _J.setZero(6, _model.dof_count);
    _J_part.setZero(3, _model.dof_count);
  }

  Point(const Point&& other)
      : _name(other._name), _model(other._model), _body_id(other._body_id),
        _position(other._position), _orientation(other._orientation)
  {
    _J.setZero(6, _model.dof_count);
    _J_part.setZero(3, _model.dof_count);
  }

  Point(const Point& other)
      : _name(other._name), _model(other._model), _body_id(other._body_id),
        _position(other._position), _orientation(other._orientation)
  {
    _J.setZero(6, _model.dof_count);
    _J_part.setZero(3, _model.dof_count);
  }

  virtual ~Point() {}

  /** returns human-readable name of a point*/
  std::string getName() const { return _name; }

  /** returns the RBDL id of a point fixed frame */
  unsigned int getBodyId() const { return _body_id; }

  /** @brief get Position in a point fixed frame*/
  const Point::Position& getPositionFixed() const { return _position; }

  const mwoibn::Vector7& getFullStateFixed()
  {

    _temp_full.head(3) = _position;
    _temp_full[3] = _orientation.x();
    _temp_full[4] = _orientation.y();
    _temp_full[5] = _orientation.z();
    _temp_full[6] = _orientation.w();

    return _temp_full;
  }

  /** @brief set new tracked point giving data in a point fixed frame*/
  void setPositionFixed(const Point::Position position)
  {
    _position = position;
  }
  void setFullStateFixed(const mwoibn::Vector7 full_state)
  {
    _position = full_state.head(3);
    _orientation.x() = full_state[3];
    _orientation.y() = full_state[4];
    _orientation.z() = full_state[5];
    _orientation.w() = full_state[6];
  }

  /** @brief get Position in a world frame */
  const Point::Position&
  getPositionWorld(const mwoibn::VectorN& joint_positions, bool update = false);

  const mwoibn::Vector7&
  getFullStateWorld(const mwoibn::VectorN& joint_positions,
                    bool update = false);

  /** @brief set new tracked point giving data in a world frame*/
  void setPositionWorld(const Point::Position position,
                        const mwoibn::VectorN& joint_positions,
                        bool update = false);
  void setFullStateWorld(const mwoibn::Vector7 state,
                         const mwoibn::VectorN& joint_positions, bool update);

  static void toFullState(mwoibn::Vector7& full_state,
                          const Point::Position& position,
                          const Point::Orientation& orientation);

  static void fromFullState(const mwoibn::Vector7& full_state,
                            Point::Position& position,
                            Point::Orientation& orientation);

  /** @brief get Position in a user-defined reference frame */
  const Point::Position&
  getPositionReference(unsigned int refernce_id,
                       const mwoibn::VectorN& joint_positions,
                       bool update = false);

  const mwoibn::Vector7&
  getFullStateReference(unsigned int refernce_id,
                        const mwoibn::VectorN& joint_positions,
                        bool update = false);
  /** @brief get Position in a user-defined reference frame
   * @note this method is slower that the respective function using RBDL id
   * number
   */

  const Point::Position&
  getPositionReference(std::string reference_name,
                       const mwoibn::VectorN& joint_positions,
                       bool update = false);

  const mwoibn::Vector7&
  getFullStateReference(std::string reference_name,
                        const mwoibn::VectorN& joint_positions,
                        bool update = false);
  /** @brief set new tracked point giving data in a user-defined reference
   * frame*/
  void setPositionReference(const Point::Position position,
                            unsigned int reference_id,
                            const mwoibn::VectorN& joint_positions,
                            bool update = false);

  void setFullStateReference(const mwoibn::Vector7 position,
                             unsigned int reference_id,
                             const mwoibn::VectorN& joint_positions,
                             bool update = false);

  void setFullStateReference(const mwoibn::Vector7 position,
                             std::string reference_name,
                             const mwoibn::VectorN& joint_positions,
                             bool update = false);
  /** @brief set new tracked point giving data in a user-defined reference frame
   *
   * @note this method is slower that the respective function using RBDL id
   *number
   */
  void setPositionReference(const Point::Position position,
                            std::string reference_name,
                            const mwoibn::VectorN& joint_positions,
                            bool update = false);

  /** @brief get Position in a point fixed frame*/
  const Point::Orientation& getOrientationFixed() const { return _orientation; }

  /** @brief set new tracked point giving data in a point fixed frame*/
  void setOrientationFixed(const Point::Orientation orientation)
  {
    _orientation = orientation;
  }

  /** @brief get orientation in a world frame as a quaternion
   */
  const Point::Orientation&
  getOrientationWorld(const mwoibn::VectorN& joint_positions,
                      bool update = false)
  {
    _temp_orientation = mwoibn::Quaternion::fromMatrix(
        getRotationWorld(joint_positions, update));
    return _temp_orientation;
  }
  /** @brief get orientation in a world frame as a quaternion
   */
  Point::Orientation getOrientationWorld(const mwoibn::VectorN& joint_positions,
                                         bool update = false) const
  {
    return mwoibn::Quaternion::fromMatrix(
        getRotationWorld(joint_positions, update));
  }
  /** @brief set new tracked point giving data in a world frame
   */
  void setOrientationWorld(const Point::Orientation orientation,
                           const mwoibn::VectorN& joint_positions,
                           bool update = false)
  {
    setRotationWorld(orientation.toMatrix(), joint_positions, update);
  }

  /** @brief get Position in a user-defined reference frame
   */
  const Point::Orientation&
  getOrientationReference(unsigned int reference_id,
                          const mwoibn::VectorN& joint_positions,
                          bool update = false)
  {
    _temp_orientation = mwoibn::Quaternion::fromMatrix(
        getRotationReference(reference_id, joint_positions, update));
    return _temp_orientation;
  }

  /** @brief get Position in a user-defined reference frame
   * @note this method is slower that the respective function using RBDL id
   * number
   */
  const Point::Orientation&
  getOrientationReference(std::string reference_name,
                          const mwoibn::VectorN& joint_positions,
                          bool update = false)
  {
    _temp_orientation = mwoibn::Quaternion::fromMatrix(
        getRotationReference(reference_name, joint_positions, update));
    return _temp_orientation;
  }

  /** @brief set new tracked point giving data in a user-defined reference
   * frame

   */
  void setOrientationReference(const Point::Orientation orientation,
                               unsigned int reference_id,
                               const mwoibn::VectorN& joint_positions,
                               bool update = false)
  {

    setRotationReference(orientation.toMatrix(), reference_id, joint_positions,
                         update);
  }

  /** @brief set new tracked point giving data in a user-defined reference frame
   *
   * @note this method is slower that the respective function using RBDL id
   *number
   */
  void setOrientationReference(const Point::Orientation orientation,
                               std::string reference_name,
                               const mwoibn::VectorN& joint_positions,
                               bool update = false)
  {
    setRotationReference(orientation.toMatrix(), reference_name,
                         joint_positions, update);
  }

  /** @brief get Position in a point fixed frame
   */
  const Point::Rotation& getRotationFixed();
  Point::Rotation getRotationFixed() const;

  /** @brief set new tracked point giving data in a point fixed frame
   */
  void setRotationFixed(const Point::Rotation rotation);

  /** @brief get Position in a world frame
   */
  const Point::Rotation&
  getRotationWorld(const mwoibn::VectorN& joint_positions, bool update = false){

    _temp_rotation = RigidBodyDynamics::CalcBodyWorldOrientation(
                         _model, joint_positions, _body_id, update).transpose()*getRotationFixed();

    return _temp_rotation;
  }

  Point::Rotation getRotationWorld(const mwoibn::VectorN& joint_positions,
                                   bool update = false) const{
    return RigidBodyDynamics::CalcBodyWorldOrientation(
          _model, joint_positions, _body_id, update).transpose()*getRotationFixed();
  }

  /** @brief set new tracked point giving data in a world frame
   */
  void setRotationWorld(const Point::Rotation rotation,
                        const mwoibn::VectorN& joint_positions,
                        bool update = false);

  /** @brief get Position in a user-defined reference frame
   */
  const Point::Rotation&
  getRotationReference(unsigned int refernce_id,
                       const mwoibn::VectorN& joint_positions,
                       bool update = false);

  /** @brief get Position in a user-defined reference frame
   * @note this method is slower that the respective function using RBDL id
   * number
   */
  const Point::Rotation&
  getRotationReference(std::string reference_name,
                       const mwoibn::VectorN& joint_positions,
                       bool update = false);

  /** @brief set new tracked point giving data in a user-defined reference
   */
  void setRotationReference(const Point::Rotation rotation,
                            unsigned int reference_id,
                            const mwoibn::VectorN& joint_positions,
                            bool update = false);

  /** @brief set new tracked point giving data in a user-defined reference frame
   *
   * @note this method is slower that the respective function using RBDL id
   *number
   */
  void setRotationReference(const Point::Rotation rotation,
                            std::string reference_name,
                            const mwoibn::VectorN& joint_positions,
                            bool update = false);

  /** @bried returnes jacobian for a linear part of a full Position of a point
   *
   * @see getPositionJacobianRows, getJacobianCols
   */
  const mwoibn::Matrix&
  getPositionJacobian(const mwoibn::VectorN& joint_positions,
                      bool update = false);
  mwoibn::Matrix
  getPositionJacobian(const mwoibn::VectorN& joint_positions,
                      bool update = false) const;
  /** returnes number of jacobian rows as returned by getPositionJacobian
   *
   * @see getPositionJacobian, getJacobianCols
   */
  unsigned int getPositionJacobianRows() const { return 3; }
  /** returnes number of jacobian cols as returned by getPositionJacobian
   *
   * @see getPositionJacobian, getPositionJacobianRows
   */
  unsigned int getJacobianCols() const { return _model.dof_count; }

  /** @bried returnes jacobian for an angular part of a full Position of a point
  */
  const mwoibn::Matrix&
  getOrientationJacobian(const mwoibn::VectorN& joint_positions,
                         bool update = false);

  mwoibn::Matrix getOrientationJacobian(const mwoibn::VectorN& joint_positions,
                         bool update = false) const;
  /** returnes number of jacobian rows as returned by getOrientationJacobian
   *
   * @see getOrientationJacobian, getOrientationJacobianCols
   */
  unsigned int getOrientationJacobianRows() const { return 3; }

  /** @bried returnes jacobian for a full Position of a point
  */
  const mwoibn::Matrix& getFullJacobian(const mwoibn::VectorN& joint_positions,
                                        bool update = false);

  /** returnes number of jacobian rows as returned by getFullJacobian
   *
   * @see getFullJacobian, getFullJacobianCols
   */
  unsigned int getFullJacobianRows() const { return 6; }

  virtual int size() const { return 3; }

protected:
  /** human-readable name of a point*/
  const std::string _name;
  /** RBDL id number of a point fixed frame */
  const unsigned _body_id;
  /** reference to a RBDL model */
  RigidBodyDynamics::Model& _model;
  mwoibn::Matrix _J;
  mwoibn::Matrix _J_part;
  Point::Position _temp_position;
  Point::Orientation _temp_orientation;
  Point::Rotation _temp_rotation;
  mwoibn::Vector7 _temp_full;

  /** keeps Position (e.x. position, orientation) of a point in its fixed frame
   */
  Point::Position _position;
  Point::Orientation _orientation;

  /** helping function to provide functinoality of calling functions through
   * frames RBDL names */
  unsigned int _checkBody(std::string body_name) const
  {
    return _checkBody(body_name, _model);
  }

  /** helping function to provide functinoality of calling functions through
   * frames RBDL names */
  unsigned int _checkBody(std::string body_name,
                          RigidBodyDynamics::Model model) const
  {

    unsigned int body_id = model.GetBodyId(body_name.c_str());

    if (body_id == std::numeric_limits<unsigned int>::max())
    {

      throw(std::invalid_argument("unknown body, " + body_name +
                                  " couldn't find it in a RBDL model"));
    }
    return body_id;
  }
};

} // namespace package
} // namespace library

#endif
