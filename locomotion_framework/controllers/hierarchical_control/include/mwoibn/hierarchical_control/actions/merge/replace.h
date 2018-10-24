#ifndef __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_MERGE__REPLACE_H
#define __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_MERGE__REPLACE_H

#include "mwoibn/hierarchical_control/maps/task_map.h"
#include "mwoibn/hierarchical_control/actions/merge/end.h"

namespace mwoibn
{
namespace hierarchical_control
{
namespace actions {

namespace merge {

//
class Replace : public End  {
//
public:
Replace(mwoibn::Matrix& P, mwoibn::VectorN& command, MergeManager& memory, maps::TaskMap& map, actions::Merge& merge, double dt, double mu);
virtual ~Replace();
const Replace& operator=(const Replace& task){
        return task;
}
virtual Local& pull();
virtual void push(Local& parent);

virtual void assign(actions::Task& t_new, actions::Task* t_old, Local* parent);
virtual void progress();

virtual Local& swapFromFront(Local& lower);
virtual Local& swapFromReplace(Local& lower);
virtual Local& swapToReplace(Local& high, Local& high_parent);
virtual Local& swapToEnd(Local& high);
// virtual void setParent(Local& parent);
virtual void run();
virtual void release();
virtual void releaseMemory();

virtual bool isParent(actions::Task& task);
virtual std::string name() {return "rep";}

protected:
virtual void _finish(Local& local){
};
Local* _parent;
};


}
}
} // namespace package
} // namespace library
#endif
