#ifndef __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_MERGE__FRONT_H
#define __MWOIBN_HIERARCHICAL_CONTROL_ACTIONS_MERGE__FRONT_H

#include "mwoibn/hierarchical_control/maps/task_map.h"
#include "mwoibn/hierarchical_control/actions/merge/local.h"
#include "mwoibn/hierarchical_control/actions/merge/front.h"
#include "mwoibn/hierarchical_control/actions/merge/merge_manager.h"
#include "mwoibn/hierarchical_control/actions/primary.h"

namespace mwoibn
{
namespace hierarchical_control
{
namespace actions {

namespace merge {

/** controlls primary actions in the replace merge stack **/
class Front : public Local  {
//
public:
Front(MergeManager& memory, maps::TaskMap& map, actions::Merge& merge);

virtual ~Front();
const Front& operator=(const Front& task){
        return task;
}
virtual Local& pull();
virtual void push(Local& parent);

virtual void release();

virtual void assign(actions::Task& t_new, actions::Task* t_old, Local* parent);
virtual void assign(actions::Task& t_new, Local* parent);

virtual void run();
virtual Local& swapFromFront(Local& lower);
virtual Local& swapFromReplace(Local& lower);

virtual bool isParent(actions::Task& task);
virtual actions::Primary& baseAction();
virtual actions::Task& action();
virtual void setParent(Local& parent);

protected:
actions::Task* _this;
Local* _parent;

};

}
}
} // namespace package
} // namespace library
#endif
