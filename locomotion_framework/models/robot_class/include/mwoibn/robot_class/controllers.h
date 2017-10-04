#ifndef ROBOT_CLASS_CONTROLLERS_H
#define ROBOT_CLASS_CONTROLLERS_H

#include "mwoibn/communication_modules/basic_controller.h"
#include <memory>

namespace mwoibn {

namespace robot_class {

class Controllers{

public:
   Controllers(){}
   virtual ~Controllers(){}

   virtual void add(std::unique_ptr<mwoibn::communication_modules::BasicController> controller){
      _controllers.push_back(std::move(controller));
   }

   virtual bool remove(int i){
       if (i < 0 || i >= _controllers.size())
         return false;

       _controllers.erase(_controllers.begin() + i);
       return true;
   }

   bool send()
   {
     bool success = true;
     for (auto& controller : _controllers)
       success = controller->send() && success;

     return success;
   }

   mwoibn::communication_modules::BasicController& controller(unsigned int id){
     if (id < _controllers.size())
       return *_controllers.at(id);
     else
       throw std::out_of_range("Given ID is beyond a vector scope");
   }


protected:
   std::vector<std::unique_ptr<mwoibn::communication_modules::BasicController>>
       _controllers;
};
}
}

#endif // CONTROLLERS_H
