#include "mwoibn/communication_modules/shared_controller.h"

bool mwoibn::communication_modules::SharedController::run()
{

  if(!_initialized) {initialize();}

    if(_position)
      mapTo(_command.position.get(), _shared[_name_position]);
    if(_velocity)
      mapTo(_command.velocity.get(), _shared[_name_velocity]);
    if(_torque)
      mapTo(_command.torque.get(), _shared[_name_torque]);


  return _initialized;
}

void mwoibn::communication_modules::SharedController::_init(std::string name){

        _name = name;

        _name_position = name + ".position";
        _name_velocity = name + ".velocity";
        _name_torque = name + ".torque";

        if(_shared.has(name))
          throw(std::invalid_argument("Shared object " + name + std::string(" already exists.")));
        if(_position && _shared.has(_name_position))
            throw(std::invalid_argument("Position interface for shared object " + name + std::string(" has already been initialized.")));
        if(_velocity && _shared.has(_name_velocity))
                throw(std::invalid_argument("Velocity interface for shared object " + name + std::string(" has already been initialized.")));
        if(_torque && _shared.has(_name_torque))
                        throw(std::invalid_argument("Torque interface for shared object " + name + std::string(" has already been initialized.")));


        mwoibn::VectorRT temp;

        mapFrom(_command.position.get(), temp);
        if(_position)
        _shared.add(_name_position, temp);

        mapFrom(_command.velocity.get(), temp);
        if(_velocity)
        _shared.add(_name_velocity, temp);

        mapFrom(_command.torque.get(), temp);

        if(_torque)
        _shared.add(_name_torque, temp);



        std::cout << "Initialized shared controller " << _name << "\n";

        std::string info = (_position)? "TRUE":"FALSE";
        std::cout << "\t position interface: " << info  << "\n";

        info = (_velocity)? "TRUE":"FALSE";
        std::cout << "\t " << "velocity interface: " <<   info  << "\n";

        info = (_torque)? "TRUE":"FALSE";
        std::cout << "\t " << "torque interface: " <<   info << "\n";
        std::cout << std::endl;

}
