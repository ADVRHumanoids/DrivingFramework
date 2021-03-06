#ifndef __MWOIBN__ROBOT_CLASS__ROBOT_POINTS__CONTACT_H
#define __MWOIBN__ROBOT_CLASS__ROBOT_POINTS__CONTACT_H

#include "mwoibn/robot_class/robot_class.h"
#include "mwoibn/robot_points/state.h"
#include "mwoibn/point_handling/wrench.h"
#include "mwoibn/point_handling/frame.h"
#include "mwoibn/point_handling/base_points_handler.h"
#include "mwoibn/communication_modules/basic_module.h"
#include "mwoibn/communication_modules/shared_point_get.h"


namespace mwoibn
{

namespace robot_points
{
/** @brief This class implements the simple contact characterized by a constant state in its end-point reference frame.
 */
class Contact : public State
{

public:
/**
 * @param model [in] reference to the RBDL model
 * @param state [in] reference to a model state vector
 * @param is_active [in] true/false whether the contact is avtive
 * @param type [in] selective flag - TO BE REMOVED
 */
template<typename Type>
Contact(Type end_frame, RigidBodyDynamics::Model& model,
        const mwoibn::robot_class::State& state, bool is_active,
        robot_class::CONTACT_TYPE type = robot_class::CONTACT_TYPE::UNKNOWN, std::string name = "")
      : State(model, state), _is_active(is_active), _frame(end_frame, model, state), _wrench(_frame.frame()),
        _type(type), _name(name) // this zero should as some kind of constant value defined
                    // in robot_class.h
{
        _resize();
        _ground_normal << 0,0,1;

        mwoibn::VectorInt ext_empty;
        mwoibn::point_handling::computeChain( 0, _model, _wrench, _chain, ext_empty);

}

Contact(RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state, YAML::Node config)
        : State(model, state), _frame([](YAML::Node contact) {return _readEndFrame(contact);}(config), model, state),
          _wrench(_frame.frame())
{
        _resize();
        _read(config);
        _ground_normal << 0,0,1;

        mwoibn::VectorInt ext_empty;
        mwoibn::point_handling::computeChain(0, _model, _wrench, _chain, ext_empty);
}

Contact(Contact&& other)
        : State(other), _is_active(other._is_active),
          _state_size(other._state_size), _frame(other._frame),
          _wrench(other._wrench, _frame.frame()), _name(other._name) //_wrench(other._wrench)
{
        _resize();
        _ground_normal << 0,0,1;

        mwoibn::VectorInt ext_empty;
        mwoibn::point_handling::computeChain( 0, _model, _wrench, _chain, ext_empty);
}

Contact(Contact& other)
        : State(other), _is_active(other._is_active),
          _state_size(other._state_size), _frame(other._frame),
          _wrench(other._wrench, _frame.frame()), _name(other._name)
{
        _resize();
        _ground_normal << 0,0,1;

        mwoibn::VectorInt ext_empty;
        mwoibn::point_handling::computeChain( 0, _model, _wrench, _chain, ext_empty);
}
using Point::operator=;
//
// virtual std::unique_ptr<mwoibn::communication_modules::CommunicationBase> generateCallback(YAML::Node config){
//   return std::unique_ptr<mwoibn::communication_modules::CommunicationBase>(nullptr);
// }



virtual void compute(){
  _point = getPosition();
}

virtual void computeJacobian(){
  getPointJacobian();
}

virtual ~Contact() {
}
/**
 * @brief Activate contact
 */
void activate() {
        _is_active = true;
}                                              // contact specific
                                               /**
                                                * @brief Deactivate contact
                                                */
void deactivate() {
        _is_active = false;
}                                              // contact specific
                                               /**
                                                * @brief Returns whether contact is active of not
                                                */
bool isActive() const {
        return _is_active;
}                                              // contact specific

bool isType(robot_class::CONTACT_TYPE type) const
{
        return _type == type;
}                                                    // contact specific
robot_class::CONTACT_TYPE type() {
        return _type;
}                                                    // contact specific

// unsigned int getBodyId() const { return _point_ptr->getBodyId(); } //Position
// // can be taken from point
unsigned int getFullJacobianRows() const {
        return _state_size;
}

virtual const mwoibn::Matrix& getPointJacobian() = 0;   // PH
virtual const mwoibn::Matrix& getWorldJacobian() = 0;   // PH

virtual const mwoibn::VectorN& getPosition() = 0;
virtual void setPosition(mwoibn::Vector3 new_state) = 0;

std::string getName(){
        return _name;
}

virtual int jacobianSize() {
        return _state_size;
}


//  const int size = 6; // PH
mwoibn::VectorInt getChain() {return _chain;}

virtual const mwoibn::VectorN& getReactionForce() = 0;

const point_handling::Wrench& wrench() const {return _wrench;}
point_handling::Wrench& wrench() {return _wrench;}

const mwoibn::Vector3& getGroundNormal() const {return _ground_normal;}
void setGroundNormal(const mwoibn::Vector3& ground_normal){_ground_normal = ground_normal;}

//const mwoibn::Matrix3& getRotationFrame(){return _frame.getRotationWorld();}
point_handling::Frame& getFrame(){return _frame;}

// add the acceleration assumptions here
virtual const mwoibn::VectorN& acceleration() = 0;
virtual const mwoibn::VectorN& velocity() = 0;

protected:


virtual void _resize(){
        _jacobian.setZero(_state_size, _state.velocity.size());
}

bool _is_active;   //!< whether a contact is active or not?

point_handling::Frame _frame;
point_handling::Wrench _wrench;
mwoibn::VectorN _acceleration, _velocity;


mwoibn::Vector3 _ground_normal;
mwoibn::VectorInt _chain;

robot_class::CONTACT_TYPE _type;
virtual void _read(YAML::Node contact);
static void _readError(std::string param);
static std::string _readEndFrame(YAML::Node contact);
int _state_size = 6;
std::string _name;

};
} // namespace package
} // namespace library
#endif // CONTACT_V2_H
