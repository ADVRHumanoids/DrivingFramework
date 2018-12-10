#ifndef __MWOIBN__MOTOR_SIDE_REFERENCE__SEA_REFERENCE_H
#define __MWOIBN__MOTOR_SIDE_REFERENCE__SEA_REFERENCE_H

#include <rbdl/rbdl.h>
#include "mwoibn/eigen_utils/eigen_utils.h"
#include "mwoibn/basic_controllers/lower_level_controller.h"
//#include "mwoibn/dynamic_models/basic_model.h"

namespace mwoibn {

namespace motor_side_reference {

class SeaReference: public mwoibn::basic_controllers::LowerLevelController {

public:
  SeaReference(mwoibn::robot_class::Robot& robot, mwoibn::basic_controllers::LowerLevelController& gravity_compensation,  mwoibn::Interface interface = "POSITION");
  virtual ~SeaReference(){}

//  void setLinkSideReference(mwoibn::VectorN reference){_link_side = reference;}
//  mwoibn::VectorN getLinkSideReference() const {return _link_side;}

  void updateStiffnessMatrix();

  virtual void compute();
  virtual const mwoibn::VectorN& update();

protected:
//  mwoibn::VectorN _link_side;
  mwoibn::Matrix _stiffness_matrix; //! Keeps inverted stiffnes matrix
  mwoibn::VectorBool _selection;
  mwoibn::basic_controllers::LowerLevelController& _gravity_compensation;
  mwoibn::PseudoInverse _stiffness_matrix_inverser;

};

} // namespace package
} // namespace library


#endif
