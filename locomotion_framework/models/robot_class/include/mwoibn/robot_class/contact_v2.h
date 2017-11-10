#ifndef ROBOT_CLASS_CONTACT_V2_H
#define ROBOT_CLASS_CONTACT_V2_H

#include "mwoibn/point_handling/raw_full_states_handler.h"
#include "mwoibn/point_handling/state_points_handler.h"
#include "mwoibn/robot_class/robot_class.h"

namespace mwoibn
{

namespace robot_class
{

typedef point_handling::StatePointsHandler<point_handling::RawFullStatesHandler,
                                           mwoibn::Vector7> BasePH;
/** @brief This class implements the simple contact characterized by a constant state in its end-point reference frame.
 */
class ContactV2 : public BasePH
{
  typedef mwoibn::Vector7 State;

public:
  /**
   * @param position [in] state of contact point in its reference frame
   * @param body_id [in] RBDL id number of a reference frame
   * @param model [in] reference to the RBDL model
   * @param positions [in] reference to a model state vector
   * @param is_active [in] true/false whether the contact is avtive
   * @param directions [in] set of constraint directions
   * @param type [in] selective flag - TO BE REMOVED
   * @param name [in] name of a contact
   */
  template<typename Type>
  ContactV2(State position, Type body_id, RigidBodyDynamics::Model& model,
            const mwoibn::VectorN& positions, bool is_active,
            mwoibn::Matrix6 directions,
            robot_class::CONTACT_TYPE type = robot_class::CONTACT_TYPE::UNKNOWN,
            std::string name = "")
      : BasePH(0, model, positions, std::vector<Type>{body_id}, {position}, {name}),
        _is_active(is_active),
        _type(type) // this zero should as some kind of constant value defined
                    // in robot_class.h
  {
    _resize();
    _initDirections(directions);

  }

  /**
   * @param point [in] constraint point
   * @param model [in] reference to the RBDL model
   * @param positions [in] reference to a model state vector
   * @param is_active [in] true/false whether the contact is avtive
   * @param directions [in] set of constraint directions
   * @param type [in] selective flag - TO BE REMOVED
   * @param name [in] name of a contact
   */
  ContactV2(point_handling::Point point, RigidBodyDynamics::Model& model,
            const mwoibn::VectorN& positions, bool is_active,
            mwoibn::Matrix6 directions,
            robot_class::CONTACT_TYPE type = robot_class::CONTACT_TYPE::UNKNOWN)
      : BasePH(0, model, positions, {point}), _is_active(is_active), _type(type)
  {
    _resize();
    _initDirections(directions);
  }

  ContactV2(RigidBodyDynamics::Model& model, const mwoibn::VectorN& positions, YAML::Node config)
      : BasePH(0, model, positions)
  {
    _resize();
    _read(config);
  }

  ContactV2(ContactV2&& other)
      : BasePH(std::move(other)), _is_active(other._is_active),
        _directions(other._directions), _state_size(other._state_size)
  {
    mwoibn::Matrix temp_directions = _directions;
    _resize();
    _directions = temp_directions;
  }

  ContactV2(ContactV2& other)
      : BasePH(other), _is_active(other._is_active),
        _directions(other._directions), _state_size(other._state_size)
  {
    mwoibn::Matrix temp_directions = _directions;
    _resize();
    _directions = temp_directions;
  }


  ~ContactV2() {}
  /**
   * @brief Activate contact
   */
  void activate() { _is_active = true; }       // contact specific
                                               /**
                                                * @brief Deactivate contact
                                                */
  void deactivate() { _is_active = false; }    // contact specific
                                               /**
                                                * @brief Returns whether contact is active of not
                                                */
  bool isActive() const { return _is_active; } // contact specific

  bool isType(robot_class::CONTACT_TYPE type) const
  {
    return _type == type;
  }                                                  // contact specific
  robot_class::CONTACT_TYPE type() { return _type; } // contact specific

  // unsigned int getBodyId() const { return _point_ptr->getBodyId(); } //Point
  // // can be taken from point
  unsigned int getFullJacobianRows() const { return _state_size; }

  virtual const mwoibn::Matrix& getPointJacobian(int i = 0); // PH

  virtual const mwoibn::Matrix& getPointJacobian(mwoibn::Matrix3 rotation_matrix,
                                          int i = 0); // contact specific

  virtual mwoibn::VectorN getPosition(); // for now it is like that, because I
                                         // still don't have a full state
                                         // handler implemented, later this
                                         // should be just a direct call to
                                         // getFullStatePosition
  virtual void setPosition(mwoibn::Vector3 new_state)
  {
    _points[0]->setPositionWorld(new_state, _positions);
  } // PH

  std::string getName(){return getPointName(0);}

  int jacobianSize() {return _state_size;}
  //  const int size = 6; // PH

protected:
//  ContactV2(int body_ref, mwoibn::Vector7 state, int body_id,
//            RigidBodyDynamics::Model& model, const mwoibn::VectorN& positions,
//            bool is_active, mwoibn::Matrix6 directions,
//            robot_class::CONTACT_TYPE type = robot_class::CONTACT_TYPE::UNKNOWN,
//            std::string name = "")
//      : BasePH(body_ref, model, positions, {body_id}, {position}, {name}),
//        _is_active(is_active),
//        _type(type) // this zero should as some kind of constant value defined
//                    // in robot_class.h
//  {
//    _initDirections(directions);
//    _resize();
//  }

//  ContactV2(std::string body_ref, mwoibn::Vector7 state,
//            std::string body_name, RigidBodyDynamics::Model& model,
//            const mwoibn::VectorN& positions, bool is_active,
//            mwoibn::Matrix6 directions,
//            robot_class::CONTACT_TYPE type = robot_class::CONTACT_TYPE::UNKNOWN,
//            std::string name = "")
//      : BasePH(body_ref, model, positions, {body_name}, {state}, {name}),
//        _is_active(is_active), _type(type)
//  {
//    _initDirections(directions);
//    _resize();
//  }

  ContactV2(std::string body_ref, point_handling::Point point,
            RigidBodyDynamics::Model& model, const mwoibn::VectorN& positions,
            bool is_active, mwoibn::Matrix6 directions,
            robot_class::CONTACT_TYPE type = robot_class::CONTACT_TYPE::UNKNOWN)
      : BasePH(body_ref, model, positions, {point}), _is_active(is_active),
        _type(type)
  {
    _resize();

    _initDirections(directions);
  }

  ContactV2(int body_ref, point_handling::Point point,
            RigidBodyDynamics::Model& model, const mwoibn::VectorN& positions,
            bool is_active, mwoibn::Matrix6 directions,
            robot_class::CONTACT_TYPE type = robot_class::CONTACT_TYPE::UNKNOWN)
      : BasePH(body_ref, model, positions, {point}), _is_active(is_active),
        _type(type)
  {
    _resize();

    _initDirections(directions);
  }

  ContactV2(RigidBodyDynamics::Model& model, const mwoibn::VectorN& positions, YAML::Node config, std::string reference)
      : BasePH(reference, model, positions)
  {
    _resize();

    _read(config);
  }

  ContactV2(RigidBodyDynamics::Model& model, const mwoibn::VectorN& positions, YAML::Node config, int reference)
      : BasePH(reference, model, positions)
  {
        _resize();
    _read(config);

  }

  virtual void _resize(){
    _jacobian.setZero(_state_size, _positions.size());
    _rotation.setZero(_state_size, _state_size);
    _transformation.setZero(_state_size, _state_size);
    _directions.setZero(_state_size, _state_size);
  }

  bool _is_active; //!< whether a contact is active or not?

  void _initDirections(mwoibn::Matrix6 directions);
  robot_class::CONTACT_TYPE _type;
  void _read(YAML::Node contact);
  void _readError(std::string param);
  mwoibn::Matrix _jacobian, _rotation, _transformation, _directions;
  int _state_size = 6;

};
} // namespace package
} // namespace library
#endif // CONTACT_V2_H
