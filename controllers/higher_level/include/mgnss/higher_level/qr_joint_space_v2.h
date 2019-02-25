#ifndef __MGNSS_HIGHER_LEVEL_QR_TO_JOINT_SPACE_V2_H
#define __MGNSS_HIGHER_LEVEL_QR_TO_JOINT_SPACE_V2_H

#include "mwoibn/robot_class/robot.h"
#include "mwoibn/hierarchical_control/tasks/controller_task.h"
#include "mgnss/higher_level/qr_task.h"

#include "mwoibn/common/logger.h"

#include "mgnss/higher_level/state_machine.h"

namespace mgnss
{

namespace higher_level
{

/*
 *  Steering version with contact point open-loop reference
 *  For now, just hardcode everythong
 */
class QRJointSpaceV2: public QrTask
{

public:
  QRJointSpaceV2(QrTask& task, const mwoibn::Matrix& jacobian, const mwoibn::VectorN& offset, mwoibn::robot_class::Robot& robot);

  ~QRJointSpaceV2(){}

virtual void init(); // alocatte all the memory
virtual void update(); // switch to joint space & solve

// virtual void solve();
virtual void log(mwoibn::common::Logger& logger);

protected:
  QrTask& _task;
  const mwoibn::Matrix& _jacobian;
  const mwoibn::VectorN& _offset;
  mwoibn::robot_class::Robot& _robot;

  virtual void _outputTransform();



};
}
}
#endif
