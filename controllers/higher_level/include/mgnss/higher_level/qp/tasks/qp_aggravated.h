#ifndef __MGNSS_HIGHER_LEVEL_QP_AGGRAVATED_H
#define __MGNSS_HIGHER_LEVEL_QP_AGGRAVATED_H

#include "mwoibn/hierarchical_control/tasks/controller_task.h"
#include "mgnss/higher_level/qp/tasks/qr_task.h"


namespace mgnss
{

namespace higher_level
{

/*
 *  Steering version with contact point open-loop reference
 *  For now, just hardcode everythong
 */
class QpAggravated: public QrTask
{

public:
  QpAggravated(int vars): QrTask(vars, 0){
  }

  ~QpAggravated(){}

virtual void add(QrTask& task){
  _tasks.push_back(&task);

}

virtual void init(); // alocatte all the memory
virtual void _update(); // switch to joint space & solve

virtual void log(mwoibn::common::Logger& logger);

protected:
  std::vector<QrTask*> _tasks;
  int _size;


};
}
}
#endif
