#ifndef ROBOT_CLASS_ACTUATORS_H
#define ROBOT_CLASS_ACTUATORS_H

#include "mwoibn/robot_class/actuator.h"
#include <memory>

namespace mwoibn{
namespace robot_class {

class Actuators{

public:
  Actuators(){}
  virtual ~Actuators(){}

  // init
  void add(std::unique_ptr<Actuator> actuator);
//  void addActuator(SeriesElasticActuator actuator);

  mwoibn::VectorN getStates();
  mwoibn::VectorN getVelocities();

  void setStates(mwoibn::VectorN state);
  void setVelocities(mwoibn::VectorN state);

  // unique actuator
  double getState(unsigned int id);
  double getVelocity(unsigned int id);
  void setState(unsigned int id, double state);
  void setVelocity(unsigned int id, double velocity);
  std::string getActuatorName(unsigned int id){return _actuators.at(id)->getName();}
  ACTUATOR_TYPE getActuatorType(unsigned int id){return _actuators.at(id)->getType();}

  // Elastic Actuators
  mwoibn::Matrix getStiffnessMatrix();
  mwoibn::Matrix getDampingMatrix();

  unsigned int size(){return _actuators.size();}
  mwoibn::VectorBool getActuationTypes(std::vector<ACTUATOR_TYPE> types);
  mwoibn::VectorInt getActuationTypes();

protected:
  std::vector<std::unique_ptr<Actuator>> _actuators;


};

} // namespace package
} // namespace library

#endif // ACTUATORS_H

