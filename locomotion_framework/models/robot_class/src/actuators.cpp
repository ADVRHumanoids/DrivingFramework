#include "mwoibn/robot_class/actuators.h"

namespace mwoibn {

namespace robot_class {


void Actuators::add(std::unique_ptr<Actuator> actuator){
  _actuators.push_back(std::move(actuator));
}


//void Actuators::addActuator(SeriesElasticActuator actuator){
//  _actuators.push_back(std::unique_ptr<Actuator>(new SeriesElasticActuator(actuator)));
//}

mwoibn::VectorN Actuators::getStates(){

  mwoibn::VectorN states(_actuators.size());

  for (int i = 0; i < _actuators.size(); i++)
    states[i] = _actuators.at(i)->getState();

  return states;
}

mwoibn::VectorN Actuators::getVelocities(){

  mwoibn::VectorN states(_actuators.size());

  for (int i = 0; i < _actuators.size(); i++)
    states[i] = _actuators.at(i)->getVelocity();

  return states;
}

void Actuators::setStates(mwoibn::VectorN state){

  for(int i = 0; i < _actuators.size(); i++)
    _actuators.at(i)->setState(state[i]);

}
void Actuators::setVelocities(mwoibn::VectorN velocities){

  for(int i = 0; i < _actuators.size(); i++)
    _actuators.at(i)->setVelocity(velocities[i]);}


mwoibn::Matrix Actuators::getStiffnessMatrix(){


  mwoibn::Matrix stiffness = mwoibn::Matrix::Zero(_actuators.size(), _actuators.size());

  for (int i = 0; i < _actuators.size(); i++){
    if ( _actuators.at(i)->getType() == ACTUATOR_TYPE::ELASTIC)
      stiffness(i,i) = _actuators.at(i).get()->getPassiveStiffness();
  }

  return stiffness;
}



mwoibn::Matrix Actuators::getDampingMatrix(){


  mwoibn::Matrix damping= mwoibn::Matrix::Zero(_actuators.size(), _actuators.size());

  for (int i = 0; i < _actuators.size(); i++){
    if ( _actuators.at(i)->getType() == ACTUATOR_TYPE::ELASTIC)
      damping(i,i) = _actuators.at(i).get()->getPassiveDamping();
  }

  return damping;

}


double Actuators::getState(unsigned int id){
  return _actuators.at(id)->getState();

}
double Actuators::getVelocity(unsigned int id){
  return _actuators.at(id)->getVelocity();
}

void Actuators::setState(unsigned int id, double state){

  _actuators.at(id)->setState(state);

}

void Actuators::setVelocity(unsigned int id, double velocity){

    _actuators.at(id)->setVelocity(velocity);
}

//Eigen::VectorXi Actuators::getActuationTypes(){

//Eigen::VectorXi state(size());

//for (int i = 0; i < size(); i++)
//  state[i] = _actuators.at(i)->getType();

//return state;

//}

mwoibn::VectorBool Actuators::getActuationTypes(std::vector<ACTUATOR_TYPE> types){

mwoibn::VectorBool state(size());

for (int i = 0; i < size(); i++){
  if (std::find(types.begin(), types.end(), _actuators.at(i)->getType()) != types.end())
    state[i] = true;
  else
    state[i] = false;

}
return state;

}


} // namespace package
} // namespace library
