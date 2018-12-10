#ifndef COMMUNICATION_MODULES_COMMUNICATION_BASE_H
#define COMMUNICATION_MODULES_COMMUNICATION_BASE_H

#include "mwoibn/common/all.h"

#include <rbdl/rbdl.h>

namespace mwoibn
{

namespace communication_modules
{

class CommunicationBase
{

public:
  CommunicationBase(int dofs): _dofs(dofs)  {  }
  CommunicationBase()  { _dofs = 0;  }

  CommunicationBase(CommunicationBase& other): _dofs(other._dofs), _initialized(other._initialized) { }
  CommunicationBase(CommunicationBase&& other): _dofs(other._dofs), _initialized(other._initialized) { }

  virtual ~CommunicationBase() {}


  //virtual mwoibn::VectorInt getSelector() const = 0;

  virtual bool update() = 0;

  int getDofs() const { return _dofs; }

  // bool is(mwoibn::Interface interface){
  //
  //   if(interface == "POSITION")
  //     return _position;
  //   if(interface == "VELOCITY")
  //     return _velocity;
  //   if(interface == "TORQUE")
  //     return _torque;
  //
  //   return false;
  // }

  virtual bool run() = 0;
  virtual bool get(mwoibn::VectorN& state){}

  virtual mwoibn::VectorInt map() const = 0; // make it fake do not have to figrue it out for all now

  virtual bool reset(){return true;}
  virtual bool initialized(){return _initialized;}
  virtual bool initialize(){
    _initialized = true;
    return initialized();}

protected:
  bool _initialized = false;
  int _dofs;
};
}
}

#endif // COMMUNICATION_MODULE_H
