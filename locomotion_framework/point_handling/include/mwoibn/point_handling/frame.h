#ifndef __MWOIBN__POINT_HANDLING__FRAME_H
#define __MWOIBN__POINT_HANDLING__FRAME_H

#include "mwoibn/point_handling/temp_base.h"
#include "mwoibn/point_handling/position.h"
#include "mwoibn/point_handling/orientation.h"
#include "mwoibn/point_handling/frame_plus.h"
#include "mwoibn/point_handling/spatial_velocity.h"


namespace mwoibn
{

namespace point_handling
{

class Frame: public TempBase<mwoibn::Vector7>
{

public:

  template<typename Body>
  Frame(Body body_id, RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state,
        std::string name = "")
      : TempBase(body_id, model, state, 7, name), _frame(body_id, model, state, name), _velocity(_frame, name)
  {   }

  template<typename Type>
  Frame(Point::Current linear, Type body_id,
        RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state,
        Orientation::O quat = Orientation::O(),  std::string name = "")
      : TempBase(body_id, model, state, 7, name), _frame(body_id, model, state, name), _velocity(_frame, name)
  {
    setLinearFixed(linear);
    setOrientationFixed(quat);
  }

  Frame( Frame&& other)
      : TempBase(other), _frame(other._frame), _temp_full(other._temp_full), _quat(other._quat),
        _pos(other._pos), _velocity(other._velocity, _frame)
  {  }

  Frame(const Frame& other)
      : TempBase(other), _temp_full(other._temp_full), _quat(other._quat),
        _pos(other._pos), _frame(other._frame), _velocity(other._velocity, _frame)
  {  }

  virtual ~Frame() {}

  /** @brief set new tracked point giving data in a point fixed frame*/
  void setLinearFixed(const Point::Current& pos)
  {
    _frame.position.setFixed(pos);
  }

  /** @brief set new tracked point giving data in a point fixed frame*/
  const Point::Current&  getLinearFixed() const
  {
    return _frame.position.getFixed();
  }

  /** @brief get Position in a world frame */
  virtual const Point::Current&
  getLinearWorld(bool update = false){return _frame.position.getWorld();}

  virtual void
  getLinearWorld(Point::Current& current, bool update = false) const {_frame.position.getWorld(current);}

  virtual void
  getLinearWorld(mwoibn::Vector3& current, bool update = false) const {_frame.position.getWorld(current);}

  virtual Point::Current
  getLinearWorld(bool update = false) const {return _frame.position.getWorld();}


  /** @brief set new tracked point giving data in a world frame*/
  virtual void setLinearWorld(const Point::Current& linear,
                        bool update = false){
    _frame.position.setWorld(linear, update);
  }

  /** @brief set new tracked point giving data in a world frame*/
  virtual void setLinearWorld(const mwoibn::Vector3& linear,
                        bool update = false){
    _frame.position.setWorld(linear, update);
  }

  /** @brief get Position in a user-defined reference frame */
  virtual Point::Current
  getLinearReference(unsigned int refernce_id, bool update = false) const {
    return _frame.position.getReference(refernce_id, update);
  }

  virtual void
  getLinearReference(Point::Current& current, unsigned int refernce_id, bool update = false){
    return _frame.position.getReference(current, refernce_id, update);
  }

  virtual void setLinearReference(const Point::Current position, unsigned int reference_id,
                        bool update = false){
    _frame.position.setReference(position, reference_id, update);
  }

  const mwoibn::Vector7& getFullStateFixed()
  {
     toFullState(_temp_full, _frame.position.getFixed(), _frame.orientation.getFixed());

    return _temp_full;
  }

  void setFullStateFixed(const mwoibn::Vector7& full_state)
  {

    fromFullState(full_state, _pos, _quat);
    _frame.position.setFixed(_pos);
    _frame.orientation.setFixed(_quat);
  }

  const mwoibn::Vector7&
  getFullStateWorld(bool update = false);

  void setFullStateWorld(const mwoibn::Vector7& state, bool update);

  static void toFullState(mwoibn::Vector7& full_state,
                          const Point::Current& linear,
                          const Orientation::O& quat);

  static void fromFullState(const mwoibn::Vector7& full_state,
                            Point::Current& linear,
                            Orientation::O& quat);

  const mwoibn::Vector7&
  getFullStateReference(unsigned int refernce_id,
                        bool update = false);
  /** @brief get Position in a user-defined reference frame
   * @note this method is slower that the respective function using RBDL id
   * number
   */

  const mwoibn::Vector7&
  getFullStateReference(std::string reference_name,
                        bool update = false);

  template<typename Body>
  void setFullStateReference(const mwoibn::Vector7 state,
                             Body reference_id,
                             bool update = false)
  {
    fromFullState(state, _pos, _quat);
    setLinearReference(_pos, reference_id, update);
    setOrientationReference(_quat, reference_id, false);
  }

  /** @brief get Position in a point fixed frame*/
  const Orientation::O& getOrientationFixed() const { return _frame.orientation.getFixed(); }

  /** @brief set new tracked point giving data in a point fixed frame*/
  void setOrientationFixed(const Orientation::O quat)
  {
    _frame.orientation.setFixed(quat);
  }

  /** @brief get orientation() in a world frame as a quaternion
   */
  const Orientation::O&
  getOrientationWorld(bool update = false)
  {
    _quat = _frame.orientation.getWorld(update);
    return _quat;
  }
  /** @brief get orientation() in a world frame as a quaternion
   */
  void getOrientationWorld(Orientation::O& angular, bool update = false) const
  {

    angular = _frame.orientation.getWorld(update);

    return;
  }
  /** @brief set new tracked point giving data in a world frame
   */
  void setOrientationWorld(const Orientation::O quat, bool update = false)
  {
    _frame.orientation.setWorld(quat, update);
  }

  /** @brief get Position in a user-defined reference frame
   */
  const Orientation::O&
  getOrientationReference(unsigned int reference_id, bool update = false)
  {
    _quat = _frame.orientation.getReference(reference_id, update);
    return _quat;
  }

  /** @brief get Position in a user-defined reference frame
   * @note this method is slower that the respective function using RBDL id
   * number
   */
  const Orientation::O&
  getOrientationReference(std::string reference_name, bool update = false)
  {
    _quat = _frame.orientation.getReference(reference_name, update);
    return _quat;
  }

  /** @brief set new tracked point giving data in a user-defined reference
   * frame

   */
  void setOrientationReference(const Orientation::O quat,
                               unsigned int reference_id,
                               bool update = false)
  {

    _frame.orientation.setReference(quat, reference_id, update);
  }

  /** @brief set new tracked point giving data in a user-defined reference frame
   *
   * @note this method is slower that the respective function using RBDL id
   *number
   */
  void setOrientationReference(const Orientation::O quat,
                               std::string reference_name,
                               bool update = false)
  {
    _frame.orientation.setReference(quat, reference_name, update);
  }

  /** @brief get Position in a point fixed frame
   */
  const Rotation::R& getRotationFixed();
  Rotation::R getRotationFixed() const;

  /** @brief set new tracked point giving data in a point fixed frame
   */
  void setRotationFixed(const Rotation::R rotation);

  /** @brief get Position in a world frame
   */
  const Rotation::R&
  getRotationWorld(bool update = false){
    return _frame.orientation.rotation().getWorld();
  }

  Rotation::R getRotationWorld(bool update = false) const{
    return _frame.orientation.rotation().getWorld();
  }

  /** @brief set new tracked point giving data in a world frame
   */
  void setRotationWorld(const Rotation::R rotation, bool update = false);

  /** @brief get Position in a user-defined reference frame
   */
  template<typename Body>
  const Rotation::R&
  getRotationReference(unsigned int refernce_id, bool update = false)
  {
    return _frame.orientation.rotation().getReference(refernce_id, update);
  }

  /** @brief set new tracked point giving data in a user-defined reference
   */
   template<typename Body>
   void setRotationReference(const Rotation::R rotation,
                            Body reference_id,
                            bool update = false)
  {
      _frame.orientation.rotation().setWorld(rotation, reference_id, update);
      _frame.orientation.synch();
  }

  /** @bried returnes jacobian for a linear part of a full Position of a point
   *
   * @see getPositionJacobianRows, getJacobianCols
   */
  const mwoibn::Matrix&
  getPositionJacobian(bool update = false);
  mwoibn::Matrix
  getPositionJacobian(bool update = false) const;
  /** returnes number of jacobian rows as returned by getPositionJacobian
   *
   * @see getPositionJacobian, getJacobianCols
   */
  unsigned int getPositionJacobianRows() const { return _velocity.linear().size(); }
  /** returnes number of jacobian cols as returned by getPositionJacobian
   *
   * @see getPositionJacobian, getPositionJacobianRows
   */
  unsigned int getJacobianCols() const { return dofs(); }

  /** @bried returnes jacobian for an angular part of a full Position of a point
  */
  const mwoibn::Matrix&
  getOrientationJacobian(bool update = false);

  mwoibn::Matrix getOrientationJacobian(bool update = false) const;
  /** returnes number of jacobian rows as returned by getOrientationJacobian
   *
   * @see getOrientationJacobian, getOrientationJacobianCols
   */
  unsigned int getOrientationJacobianRows() const { return _velocity.angular().size(); }

  /** @bried returnes jacobian for a full Position of a point
  */
  const mwoibn::Matrix& getFullJacobian(bool update = false);

  /** returnes number of jacobian rows as returned by getFullJacobian
   *
   * @see getFullJacobian, getFullJacobianCols
   */
  unsigned int getFullJacobianRows() const { _velocity.size(); }

  //virtual int size() const { return 3; }

  point_handling::Position& position(){return _frame.position;}
  point_handling::FramePlus& frame(){return _frame;}

  point_handling::Orientation& orientation(){return _frame.orientation;}
  point_handling::SpatialVelocity& velocity() {return _velocity;}
/*
  const point_handling::Position& const position(){return position();}
  const point_handling::Orientation& const orientation()(){return orientation();}
  const point_handling::SpatialVelocity& const velocity() {return _velocity;}
*/

protected:

  mwoibn::Vector7 _temp_full;
  Orientation::O _quat;
  Point::Current _pos;
  point_handling::FramePlus _frame;
  point_handling::SpatialVelocity _velocity;

};

} // namespace package
} // namespace library

#endif
