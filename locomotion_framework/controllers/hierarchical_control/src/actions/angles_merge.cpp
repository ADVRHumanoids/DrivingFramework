#include "mwoibn/hierarchical_control/actions/angles_merge.h"


mwoibn::hierarchical_control::actions::AnglesMerge::AnglesMerge(actions::Compute& main_task, actions::Compute& secondary_task, hierarchical_control::State& state, mwoibn::Scalar eps, mwoibn::Scalar p, std::vector<mwoibn::hierarchical_control::tasks::Angle>& caster_tasks, mwoibn::hierarchical_control::tasks::Constraints& constraints, mwoibn::robot_class::Robot& robot)  : Merge( main_task,  secondary_task,   state, eps, p, constraints, robot), _caster_tasks(caster_tasks){

}

void mwoibn::hierarchical_control::actions::AnglesMerge::_check(){

        for(int i = 0; i < _task.getTaskSize(); i++) {
                _current_id[i] = _primary.getJacobian().row(i).cwiseAbs().maxCoeff() > _eps;
//std::cout << "sing\t" << _primary.getJacobian().row(i).cwiseAbs().maxCoeff() << "\t";
              }

            //  std::cout <<  _primary.getJacobian().row(0).cwiseAbs().maxCoeff() << "\t";

}
