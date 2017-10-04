#include "mwoibn/robot_class/actuator.h"

namespace mwoibn
{

namespace robot_class
{

Actuator::Actuator(YAML::Node data)
{
  bool verify = (data["Bm"] && data["nu"]) || data["B"];
  verify = verify && ((data["phim"] && data["nu"]) || data["phi"]);
  verify = verify && data["name"];

  if (!verify)
  {

    std::stringstream errMsg;
    errMsg << "Couldn't load actuator '" << data["name"].as<std::string>()
           << "." << std::endl;
    errMsg << "At least on of the following parameters is missing\n: "
              "motro-side inertia (Bm\\B),\n motor-side damping (phim\\phi) or "
              "gearing (nu)" << std::endl;
    throw(std::invalid_argument(errMsg.str().c_str()));
  }

  double nu = 1;
  if (data["nu"])
    nu = data["nu"].as<double>();

  if (data["B"])
    _motor_inertia = data["B"].as<double>();
  else
    _motor_inertia = data["Bm"].as<double>() * nu * nu;

  if (data["phi"])
    _motor_damping = data["phi"].as<double>();
  else
    _motor_damping = data["phim"].as<double>() * nu * nu;

  _name = data["name"].as<std::string>();
}

SeriesElasticActuator::SeriesElasticActuator(YAML::Node data)
    : Actuator(data)
{
  _setType(ACTUATOR_TYPE::ELASTIC);

  bool verify = data["Ks"];
  verify = verify && data["D"];

  if (!verify)
  {
    std::stringstream errMsg;
    errMsg << "Couldn't load actuator '" << data["name"].as<std::string>()
           << "." << std::endl;
    errMsg << "At least on of the following parameters is missing\n: elastic "
              "element stiffness (Ks),\n elastic element damping (D)"
           << std::endl;
    throw(std::invalid_argument(errMsg.str().c_str()));
  }

  _elastic_stiffness = data["Ks"].as<double>();
  _elastic_damping = data["D"].as<double>();
}

} // namespace package
} // namespace library
