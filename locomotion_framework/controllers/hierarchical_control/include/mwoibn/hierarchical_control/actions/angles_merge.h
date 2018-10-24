#ifndef __MWOIBN_HIERARCHICAL_CONTROL_TASKS__ANGLES_MERGE_H
#define __MWOIBN_HIERARCHICAL_CONTROL_TASKS__ANGLES_MERGE_H

#include "mwoibn/hierarchical_control/actions/merge.h"
#include "mwoibn/hierarchical_control/tasks/angle.h"

namespace mwoibn
{
namespace hierarchical_control
{
namespace actions {

class AnglesMerge : public Merge
{
public:
AnglesMerge(actions::Compute& main_task, actions::Compute& secondary_task, hierarchical_control::State& state, mwoibn::Scalar eps, mwoibn::Scalar p, std::vector<mwoibn::hierarchical_control::tasks::Angle>& caster_task, mwoibn::hierarchical_control::tasks::Constraints& constraints, mwoibn::robot_class::Robot& robot);

virtual ~AnglesMerge(){
}

protected:
//
virtual void _check();

//
virtual void _reset(int i){

  //std::cout << "reset" << _caster_tasks[i].getError() << "\t" << _caster_tasks[i].getReference() << "\t" << _caster_tasks[i].getCurrent() << std::endl;

  _caster_tasks[i].setReference(_caster_tasks[i].getCurrent());

  //std::cout << _caster_tasks[i].getError() << "\t" << _caster_tasks[i].getReference() << "\t" << _caster_tasks[i].getCurrent() << std::endl;

}

virtual void _read(){
  /*std::cout << "read" << std::endl;
  for(auto& task: _caster_tasks){
    std::cout << task.getError() << "\t" << task.getReference() << "\t" << task.getCurrent() << std::endl;
  }*/
}

protected:
  std::vector<mwoibn::hierarchical_control::tasks::Angle>& _caster_tasks;

};



}
} // namespace package
} // namespace library
#endif
