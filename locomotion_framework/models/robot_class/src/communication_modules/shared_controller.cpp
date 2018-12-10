#include "mwoibn/communication_modules/shared_controller.h"

bool mwoibn::communication_modules::SharedController::run()
{

  if(!_initialized) {initialize();}

    if(_position)
      mapTo(_command.position.get(), _shared[_name+".position"]);
    if(_velocity)
      mapTo(_command.velocity.get(), _shared[_name+".velocity"]);
    if(_torque)
      mapTo(_command.torque.get(), _shared[_name+".torque"]);


  return _initialized;
}

void mwoibn::communication_modules::SharedController::_init(std::string name){
        if(_shared.has(name))
          throw(std::invalid_argument("Shared object " + name + std::string(" already exists.")));
        if(_position && _shared.has(name+".position"))
            throw(std::invalid_argument("Position interface for shared object " + name + std::string(" has already been initialized.")));
        if(_velocity && _shared.has(name+".velocity"))
                throw(std::invalid_argument("Velocity interface for shared object " + name + std::string(" has already been initialized.")));
        if(_torque && _shared.has(name+".torque"))
                        throw(std::invalid_argument("Torque interface for shared object " + name + std::string(" has already been initialized.")));


        mwoibn::VectorRT temp;

        mapFrom(_command.position.get(), temp);
        _shared.add(name+".position", temp);

        mapFrom(_command.velocity.get(), temp);
        _shared.add(name+".velocity", temp);

        mapFrom(_command.torque.get(), temp);
        _shared.add(name+".torque", temp);


        _name = name;

        std::cout << "Initialized shared controller " << _name << "\n";

        std::string info = (_position)? "TRUE":"FALSE";
        std::cout << "\t position interface: " << info  << "\n";

        info = (_velocity)? "TRUE":"FALSE";
        std::cout << "\t " << "velocity interface: " <<   info  << "\n";

        info = (_torque)? "TRUE":"FALSE";
        std::cout << "\t " << "torque interface: " <<   info << "\n";
        std::cout << std::endl;

}
