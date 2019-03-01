#ifndef __MGNSS_HIGHER_LEVEL_QR_TO_JOINT_SPACE_H
#define __MGNSS_HIGHER_LEVEL_QR_TO_JOINT_SPACE_H

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
class QRJointSpace: public QrTask
{

public:
  QRJointSpace(QrTask& task, const mwoibn::Matrix& jacobian, mwoibn::robot_class::Robot& robot);

  ~QRJointSpace(){}

virtual void init(); // alocatte all the memory
virtual void _update(); // switch to joint space & solve

// virtual void solve();
virtual void log(mwoibn::common::Logger& logger);

protected:
  QrTask& _task;
  const mwoibn::Matrix& _jacobian;
  mwoibn::robot_class::Robot& _robot;
};
}
}
#endif
