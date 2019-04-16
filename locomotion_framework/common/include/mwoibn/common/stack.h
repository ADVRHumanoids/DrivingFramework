#ifndef __MWOIBN__COMMON__STACK_H
#define __MWOIBN__COMMON__STACK_H

#include "mwoibn/common/types.h"
#include "mwoibn/common/interfaces.h"
#include "mwoibn/common/pipe.h"

//#include <rbdl/rbdl.h>
namespace mwoibn::common
{

  template <typename Key, typename Type>
  class Stack
  {
  public:
    Stack() = default;
    Stack(const  Stack& other) = default;
    Stack(Stack&& other) = default;
    Stack(std::initializer_list<Key> names) {
        for(auto& name: names)
          _interfaces[name] = Type();
    }


  bool add(Key name){
    if(_interfaces.count(name))
      return false;

    _interfaces[name] = Type();
    return true;
  }

  bool add(Key name, int size){
    if(_interfaces.count(name))
      return false;

    _interfaces[name] = Type(size);
    return true;
  }


  bool has(const Key& name){
    return _interfaces.count(name);
  }

  void init(int dofs){
      for(auto& name: _interfaces)
        name.second.init(dofs);
  }

  const Type& operator[] (const Key& name) const {
        return _interfaces.at(name);
  }
  Type& operator[] (const Key& name) {
        return _interfaces.at(name);
  }



  protected:
    std::map<Key, Type> _interfaces;

  };
}

#endif // STATE_H
