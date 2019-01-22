#ifndef ROBOT_CLASS_ACTUATOR_H
#define ROBOT_CLASS_ACTUATOR_H

#include "mwoibn/robot_class/robot_class.h"
// #include <rbdl/rbdl.h>

namespace mwoibn
{
namespace robot_class
{

class Actuator
{

public:
  Actuator(ACTUATOR_TYPE type, double motor_inertia = 0,
           double motor_damping = 0, std::string name = "")
      : _motor_inertia(motor_inertia), _motor_damping(motor_damping),
        _name(name), _type(type)
  {
  }

  Actuator(YAML::Node data);

  virtual ~Actuator() {}

  double getState() { return _state; }
  double getVelocity() { return _velocity; }

  void setState(double state) { _state = state; }
  void setVelocity(double velocity) { _velocity = velocity; }

  double getMotorInertia() { return _motor_inertia; }
  double getMotorDamping() { return _motor_damping; }

  virtual double getPassiveStiffness() { return 0; }
  virtual double getPassiveDamping() { return 0; }

  ACTUATOR_TYPE getType() { return _type; }

  std::string getName() { return _name; }

protected:
  double _motor_inertia;
  double _motor_damping;

  double _state;
  double _velocity;

  void _setType(ACTUATOR_TYPE type) { _type = type; }

  std::string _name;

  ACTUATOR_TYPE _type;
};

class SeriesElasticActuator : public Actuator
{

public:
  SeriesElasticActuator(double motor_inertia, double motor_damping,
                        double elastic_stiffness, double elastic_damping,
                        std::string name = "")
      : Actuator(ACTUATOR_TYPE::ELASTIC, motor_inertia, motor_damping, name),
        _elastic_stiffness(elastic_stiffness), _elastic_damping(elastic_damping)
  {
  }

  SeriesElasticActuator(YAML::Node data);

  virtual ~SeriesElasticActuator() {}

  double getPassiveStiffness() { return _elastic_stiffness; }
  double getPassiveDamping() { return _elastic_damping; }

protected:
  double _elastic_stiffness;
  double _elastic_damping;
};
} // namespace package
} // namespace library
#endif // ACTUATOR_H
