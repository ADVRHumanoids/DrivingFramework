#ifndef __MWOIBN__POINT_HANDLING__BASE_H
#define __MWOIBN__POINT_HANDLING__BASE_H

#include "mwoibn/point_handling/temp_base.h"

namespace mwoibn
{

namespace point_handling
{

template<typename Type>
class Base: public TempBase<Type>
{

public:
  Base(unsigned int body_id, RigidBodyDynamics::Model& model,
       const mwoibn::robot_class::State& state, unsigned int size, std::string name = "")
      : TempBase<Type>(body_id, model, state, size, name)
  {
  }

  Base(std::string body_name, RigidBodyDynamics::Model& model,
       const mwoibn::robot_class::State& state, unsigned int size, std::string name = "")
      : TempBase<Type>(body_name, model, state, size, name)
  {
  }

  Base(Type current, unsigned int body_id,
        RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state,
        unsigned int size, std::string name = "")
      : TempBase<Type>(body_id, model, state, size, name), _current_fixed(current), _temp_world(current)
  {  }

  Base(Type current, std::string body_name,
        RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state,
        unsigned int size, std::string name = "")
      : TempBase<Type>(body_name, model, state, size, name), _current_fixed(current), _temp_world(current)
  {  }

  Base( Base&& other)
      : TempBase<Type>(other), _current_fixed(other._current_fixed), _temp_world(other._temp_world)
  {  }

  Base(const Base& other)
      : TempBase<Type>(other), _current_fixed(other._current_fixed), _temp_world(other._temp_world)
  {  }

  template<typename Source>
  Base( Source&& other, int size, std::string name = "")
      : TempBase<Type>(other, size, name)
  {  }

  template<typename Source>
  Base(const Source& other, int size, std::string name = "")
      : TempBase<Type>(other, size, name)
  {  }

  template<typename Source>
  Base(Type current,  Source&& other, int size, std::string name = "")
      : TempBase<Type>(other, size, name), _current_fixed(current), _temp_world(current)
  {  }

  template<typename Source>
  Base(Type current, const Source& other, int size, std::string name = "")
      : TempBase<Type>(other, size, name), _current_fixed(current), _temp_world(current)
  {  }



  virtual ~Base() {}

  /** @brief get Position in a point fixed frame*/
  virtual const Type& getFixed() const { return _current_fixed; }

  /** @brief set new tracked point giving data in a point fixed frame*/
  //virtual void setFixed(const Type& current){ _current_fixed.noalias() = current; }
  virtual void setFixed(const Type& current) = 0;
  /** @brief get Position in a world frame */
  virtual const Type&
  getWorld(bool update = false) = 0;

  virtual Type
  getWorld(bool update = false) const = 0;
  /** @brief set new tracked point giving data in a world frame*/
  virtual void setWorld(const Type& current,
                        bool update = false) = 0;

  /** @brief get Position in a user-defined reference frame */
  virtual Type
  getReference(unsigned int refernce_id, bool update = false) const = 0;

  Type
  getReference(std::string reference_name, bool update = false) const{
    return getReference(TempBase<Type>::_checkBody(reference_name), update);
  }

  virtual void setReference(const Type& current,
                            unsigned int reference_id,
                            bool update = false) = 0;

  void setReference(const Type& current,
                          std::string reference_name,
                          bool update = false){
                            setReference(current, TempBase<Type>::_checkBody(reference_name), update);
                          }


protected:
  Type _current_fixed, _temp_world;

};

} // namespace package
} // namespace library

#endif
