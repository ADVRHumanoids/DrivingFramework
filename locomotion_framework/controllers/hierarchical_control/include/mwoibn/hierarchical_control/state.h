#ifndef __MWOIBN_HIERARCHICAL_CONTROL__STATE_H
#define __MWOIBN_HIERARCHICAL_CONTROL__STATE_H

#include "mwoibn/common/types.h"
#include "mwoibn/hierarchical_control/maps/actions_map.h"
#include "mwoibn/hierarchical_control/memory/manager.h"
namespace mwoibn
{
namespace hierarchical_control
{
struct State {
public:
        State(mwoibn::VectorN& command, mwoibn::Matrix& P, maps::ActionsMap& map, memory::Manager& memory, double dt, unsigned int dofs) : command(command), P(P), map(map), memory(memory), dt(dt), dofs(dofs){
        }
        mwoibn::VectorN& command;
        mwoibn::Matrix& P;
        maps::ActionsMap& map;
        memory::Manager& memory;
        double dt;
        unsigned int dofs;
};
}
}
#endif
