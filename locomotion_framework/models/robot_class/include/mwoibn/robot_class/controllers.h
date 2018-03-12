#ifndef ROBOT_CLASS_CONTROLLERS_H
#define ROBOT_CLASS_CONTROLLERS_H

#include "mwoibn/communication_modules/basic_controller.h"
#include <memory>

namespace mwoibn
{

namespace robot_class
{

class Controllers
{

public:
  Controllers() {}
  virtual ~Controllers() {}

  virtual void
  add(std::unique_ptr<mwoibn::communication_modules::BasicController>
          controller,
      std::string name = "")
  {
    _controllers.push_back(std::move(controller));
    _names.push_back(name);
  }

  virtual bool remove(int i)
  {
    if (i < 0 || i >= _controllers.size())
      return false;

    _controllers.erase(_controllers.begin() + i);
    _names.erase(_names.begin() + i);

    return true;
  }

  bool send()
  {
    bool success = true;
    for (auto& controller : _controllers)
      success = controller->send() && success;

    return success;
  }

  mwoibn::communication_modules::BasicController& controller(unsigned int id)
  {
    if (id < _controllers.size())
      return *_controllers.at(id);
    else
      throw std::out_of_range("Given ID is beyond a vector scope");
  }

  int getId(std::string name)
  {

    auto name_ptr =
        std::find_if(_names.begin(), _names.end(), [&name](std::string names)
                     {
                       return names == name;
                     });
    if (name_ptr == _names.end())
    {
      throw std::invalid_argument("Couldn't find controller " + name);
    }

    return std::distance(_names.begin(), name_ptr);
  }

  mwoibn::communication_modules::BasicController& controller(std::string name)
  {
    return *_controllers.at(getId(name));
  }

  void remove(std::string name) { remove(getId(name)); }

   void reset(){
     for (auto& controller : _controllers)
       controller->reset();

     return;
   }


protected:
  std::vector<std::unique_ptr<mwoibn::communication_modules::BasicController>>
      _controllers;
  std::vector<std::string> _names;
};
}
}

#endif // CONTROLLERS_H
