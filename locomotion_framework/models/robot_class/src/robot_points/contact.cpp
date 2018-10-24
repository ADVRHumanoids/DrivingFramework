#include "mwoibn/robot_points/contact.h"


void mwoibn::robot_points::Contact::_read(YAML::Node contact)
{
  if(contact["active"] && contact["active"].as<bool>())
    _is_active = true;
  else
    _is_active = false;

  _name = contact["name"] ? contact["name"].as<std::string>() : "";

}

void mwoibn::robot_points::Contact::_readError(std::string param)
{
  std::stringstream errMsg;
  errMsg << "There was an error while reading the " << param << std::endl;
  throw(std::invalid_argument(errMsg.str().c_str()));
}

std::string mwoibn::robot_points::Contact::_readEndFrame(YAML::Node contact){
  if (!contact["end_frame"])
    _readError("end_frame");
  return contact["end_frame"].as<std::string>();
}
