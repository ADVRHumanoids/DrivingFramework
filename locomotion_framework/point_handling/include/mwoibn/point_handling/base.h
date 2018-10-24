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
    //_current.setZero(size);
  }

  Base(std::string body_name, RigidBodyDynamics::Model& model,
       const mwoibn::robot_class::State& state, unsigned int size, std::string name = "")
      : TempBase<Type>(body_name, model, state, size, name)
  {
    //_current.setZero(size);
  }

  Base(Type current, unsigned int body_id,
        RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state,
        unsigned int size, std::string name = "")
      : TempBase<Type>(body_id, model, state, size, name), _current(current), _temp_current(current)
  {  }

  Base(Type current, std::string body_name,
        RigidBodyDynamics::Model& model, const mwoibn::robot_class::State& state,
        unsigned int size, std::string name = "")
      : TempBase<Type>(body_name, model, state, size, name), _current(current), _temp_current(current)
  {  }

  Base(const Base&& other)
      : TempBase<Type>(other), _current(other._current), _temp_current(other._temp_current)
  {  }

  Base(const Base& other)
      : TempBase<Type>(other), _current(other._current), _temp_current(other._temp_current)
  {  }

  template<typename Source>
  Base(const Source&& other, int size, std::string name = "")
      : TempBase<Type>(other, size, name)
  {  }

  template<typename Source>
  Base(const Source& other, int size, std::string name = "")
      : TempBase<Type>(other, size, name)
  {  }

  template<typename Source>
  Base(Type current, const Source&& other, int size, std::string name = "")
      : TempBase<Type>(other, size, name), _current(current), _temp_current(current)
  {  }

  template<typename Source>
  Base(Type current, const Source& other, int size, std::string name = "")
      : TempBase<Type>(other, size, name), _current(current), _temp_current(current)
  {  }



  virtual ~Base() {}

  /** @brief get Position in a point fixed frame*/
  const Type& getFixed() const { return _current; }

  /** @brief set new tracked point giving data in a point fixed frame*/
  void setFixed(const Type& current){ _current.noalias() = current; }

  /** @brief get Position in a world frame */
  virtual const Type&
  getWorld(bool update = false) = 0;

  virtual Type
  getWorld(bool update = false) const = 0;
  /** @brief set new tracked point giving data in a world frame*/
  virtual void setWorld(const Type& current,
                        bool update = false) = 0;

  /** @brief get Position in a user-defined reference frame */
  virtual const Type&
  getReference(unsigned int refernce_id, bool update = false) = 0;

  const Type&
  getReference(std::string reference_name, bool update = false);

  virtual void setReference(const Type& current,
                            unsigned int reference_id,
                            bool update = false) = 0;

  void setReference(const Type& current,
                          std::string reference_name,
                          bool update = false);


protected:
  Type _current, _temp_current;

};

} // namespace package
} // namespace library

#endif
