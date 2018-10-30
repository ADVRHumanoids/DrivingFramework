#ifndef ROBOT_CLASS_CONTACTS_H
#define ROBOT_CLASS_CONTACTS_H

#include "mwoibn/robot_points/contact.h"
#include <rbdl/rbdl.h>
#include "mwoibn/point_handling/position.h"
#include <memory>
#include "mwoibn/eigen_utils/eigen_utils.h"

namespace mwoibn
{
namespace robot_class
{

/** @brief Keeps all the contact data and probides acces to individual contacts
 * **/
class Contacts
{

public:
Contacts(unsigned int dofs) : _dofs(dofs) {
        _boolean_checks.setConstant(false, _dofs);

}
virtual ~Contacts() {
}

virtual void add(std::unique_ptr<robot_points::Contact> contact)
{
        _contacts.push_back(std::move(contact));
        resize();
}   // NRT

virtual void resize(){
        _jacobian = mwoibn::Matrix::Zero(jacobianRows(), _dofs);
        _positions = mwoibn::VectorN::Zero(_contacts.size() * 7);
}   // NRT

/**
 * @brief removes contact from a stack
 *
 * function returnes false if given number is negative or it is bigger that
 ****current stack size
 *
 * @param i - contact ID number
 *
 * @return whether a contact was removed succesfully, false will be returned
 ****if given reference number exceeds size of contacts vector, or negative value
 ****was send
 *
 */
virtual bool remove(int i);   // NRT

/**
 * @brief Gives number of all considered contact points
 *
 */
int size() const {
        return _contacts.size();
}                                               // RT?

/**
 * @brief return the index of a Contact with given name
 *
 * @param name Name of a contact
 *
 * @return index in the _contacts vector, -1 if no result has been found
 */
int getId(std::string name) const;   // RT?

/**
 * @brief returns a copy of a Contact with given ID
 *
 * @param id Id of a given contact
 *
 * @@throws std::out_of_range if ID is beyond the scope of a _contacts vector
 */
robot_points::Contact& contact(unsigned int id);   // RT
/**
 * @brief Check which contacts are active and returns theirs ids
 *
 * @param names optional parameter if also names should be returned
 *
 */
std::vector<unsigned int>
getActive(std::vector<std::string>* names = nullptr) const;   // NRT change it to work
                                                              // by type not name,
                                                              // no need for
                                                              // pointers. it can
                                                              // work through value
                                                              /**
                                                               * @brief Check which contacts are inactive and returns theirs ids
                                                               *
                                                               * @param names optional parameter if also names should be returned
                                                               *
                                                               */
std::vector<unsigned int>
getInactive(std::vector<std::string>* names = nullptr) const;   // NRT

/**
 * @brief Returns how many contacts is currently active
 */
unsigned int sizeActive() const;   // RT?
/**
 * @brief Returns how many contacts is currently active
 */
unsigned int sizeInactive() const;   // RT?

/**
 * @brief Returns constraints Jacobian for current state of the robot
 */
const mwoibn::Matrix& getJacobian();   // RT - change to get and update
const mwoibn::Matrix& getWorldJacobian();   // RT - change to get and update

/**
 * @brief Returns constraints Jacobian for current state of the robot
 */
mwoibn::Matrix getMinimumJacobian();   // NRT?

/**
 * @brief Returns constraints Jacobian for current state of the robot
 */
std::vector<mwoibn::Matrix> getJacobians();   // NRT - to RT
std::vector<mwoibn::Matrix> getWorldJacobians();   // NRT - to RT

/**
 * @brief Returns constraints Jacobian for current state of the robot
 */
std::vector<mwoibn::Matrix> getMinimumJacobians();   // NRT

/**
 * @brief Returns vector of current positions of all contact point in a world
 * frame
 */
const mwoibn::VectorN& getPosition();   // NRT - to RT

/**
 * @brief Returns vector of current positions of all contact point in a world
 * frame
 */
std::vector<mwoibn::VectorN> getPositions();   // NRT - to RT
mwoibn::VectorBool getTypes(std::vector<CONTACT_TYPE> types);   // NRT

const mwoibn::VectorBool& getDofs();   // RT
const mwoibn::VectorBool& getActiveDofs();   // RT
const mwoibn::VectorBool& getInactiveDofs();   // RT

int jacobianRows() const {
        int i = 0;
        for (auto& contact : _contacts)
                i+= contact->jacobianSize();
        return i;
}
int jacobianCols() const {
        return _dofs;
}

std::vector<std::unique_ptr<mwoibn::robot_points::Contact>>::iterator begin(){return _contacts.begin();}
std::vector<std::unique_ptr<mwoibn::robot_points::Contact>>::iterator end(){return _contacts.end();}

std::vector<std::unique_ptr<mwoibn::robot_points::Contact>>::const_iterator begin() const {return _contacts.begin();}
std::vector<std::unique_ptr<mwoibn::robot_points::Contact>>::const_iterator end() const {return _contacts.end();}

protected:
std::vector<std::unique_ptr<mwoibn::robot_points::Contact> > _contacts;
mwoibn::Matrix _jacobian;
mwoibn::VectorN _positions;
unsigned int _dofs;
mwoibn::VectorBool _boolean_checks;


///@}
};
} // namespace package
} // namespace library
#endif // CONTACTS_H
