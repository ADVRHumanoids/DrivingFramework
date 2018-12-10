#include "mwoibn/communication_modules/shared_point_setter.h"

bool mwoibn::communication_modules::SharedPointSetter::run()
{

  if(!_initialized) {initialize();}

    if(_world)
      _shared[_name+".world"].head<6>() = _point.getWorld();
    else
      _shared[_name+".fixed"].head<6>() = _point.getFixed();


  return _initialized;
}

void mwoibn::communication_modules::SharedPointSetter::_init(std::string name){
          if(_shared.has(name))
            throw(std::invalid_argument(__PRETTY_FUNCTION__ + std::string("Shared object ") + name + std::string(" already exists.")));
          if(_world && _shared.has(name+".world"))
            throw(std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": World interface for shared object ") + name + std::string(" has already been initialized.")));
          if(!_world && _shared.has(name+".fixed"))
              throw(std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": Fixed interface for shared object ") + name + std::string(" has already been initialized.")));

          mwoibn::VectorRT temp;

          if(_world){
            temp.head<6>() = _point.getWorld();
            _shared.add(name+".world", temp);
          }
          else{
            temp.head<6>() = _point.getFixed();
            _shared.add(name+".fixed", temp);
          }

          _name = name;

          std::cout << "Initialized shared point " << _name << " ";

          std::string info = (_world)? "world":"fixed";
          std::cout << "interface: " << info  << std::endl;
  }
