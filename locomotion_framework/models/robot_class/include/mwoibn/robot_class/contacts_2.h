#ifndef __MWOIBN__ROBOT_CLASS__CONTACTS_2_H
#define __MWOIBN__ROBOT_CLASS__CONTACTS_2_H

#include "mwoibn/robot_class/contacts.h"

namespace mwoibn
{
namespace robot_class
{

/** @brief Keeps all the contact data and probides acces to individual contacts
 * **/
class Contacts2: public Contacts
{

public:
Contacts2(unsigned int dofs) : Contacts(dofs) {
}

virtual ~Contacts2() = default;


virtual void add(std::unique_ptr<robot_points::Contact> new_contact)
{
      if(!new_contact->getName().empty() && _names.count(new_contact->getName()) )
        throw std::runtime_error(std::string("Could not add contact to the stack. Contact ")+new_contact->getName()+std::string(" has already been defined."));

      _contacts.push_back(std::move(new_contact));
      if(!_contacts.back()->getName().empty())
        _names[_contacts.back()->getName()] = _contacts.back().get();
      resize();
}   // NRT

virtual void toGroup(const std::string& name, const std::string& group){
  if (!_names.count(name))
    throw std::invalid_argument(std::string("Requested contact ")+name+std::string(" does not exists."));

  if (!_groups.count(group)) {_groups[group].push_back(_names[name]); return;}

  auto i = ranges::find_if(_groups[group], [&](auto contact) -> bool { return contact->getName() == name; } );
  if(i == ranges::end(_groups[group])) _groups[group].push_back(_names[name]);

}


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
virtual bool remove(int i){
  _names.erase(_contacts[i]->getName());
  for(auto& group: _groups)
    ranges::remove_if(group.second, [&](auto contact) -> bool { return contact == _contacts[i].get(); } );

  Contacts::remove(i);
}

virtual const mwoibn::robot_points::Contact& operator[] (int i) const {
        return *_contacts.at(i);
}

virtual mwoibn::robot_points::Contact& operator[](int i) {
        return *_contacts.at(i);
}

virtual const mwoibn::robot_points::Contact& operator[] (const std::string& name) const {
        return *_names.at(name);
}

virtual mwoibn::robot_points::Contact& operator[](const std::string& name) {
        return *_names.at(name);
}

virtual const std::vector<mwoibn::robot_points::Contact*>& group(const std::string& name) const {
  return _groups.at(name);
}

virtual std::vector<mwoibn::robot_points::Contact*>& group(const std::string& name) {
     return _groups.at(name);
}

bool hasGroup(std::string group){return _groups.count(group);}

protected:
std::map<std::string, mwoibn::robot_points::Contact* > _names;
std::map<std::string, std::vector<mwoibn::robot_points::Contact*> > _groups;

///@}
};
} // namespace package
} // namespace library
#endif // CONTACTS_H
