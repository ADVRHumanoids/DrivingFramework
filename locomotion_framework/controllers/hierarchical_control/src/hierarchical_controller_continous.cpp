#include "mwoibn/hierarchical_control/hierarchical_controller_continous.h"
#include <cmath>

void mwoibn::hierarchical_control::HierarchicalControllerContinous::addTask(ControllerTask* new_task, mwoibn::VectorN gain, int i){
  mwoibn::hierarchical_control::HierarchicalController::addTask(new_task, gain, i);
    if ((i < 0) || (i >= _tasks_ptr.size())){
		_last_tasks_ranks.push_back(0); // set as zero i.e. in last updated it has not been executed
	}
	else{
		_last_tasks_ranks.insert(_last_tasks_ranks.begin()+i,0);
		std::fill(_last_tasks_ranks.begin()+i+1, _last_tasks_ranks.end(), 0);
	}

}

void mwoibn::hierarchical_control::HierarchicalControllerContinous::removeTask(int i){
  mwoibn::hierarchical_control::HierarchicalController::removeTask(i);
	if ((i >= 0) && (i < _tasks_ptr.size())){
		_last_tasks_ranks.erase(_last_tasks_ranks.begin()+i);
		std::fill(_last_tasks_ranks.begin()+i, _last_tasks_ranks.end(), 0);
	}

}

mwoibn::VectorN mwoibn::hierarchical_control::HierarchicalControllerContinous::update(){
	for (auto& task: _tasks_ptr)
	{
		task->update();
	}
	_previous_joint_states = _joint_states;
  _joint_states = _robot->state.get(mwoibn::robot_class::INTERFACE::VELOCITY);
  compute();

  return getCommand();

}

void mwoibn::hierarchical_control::HierarchicalControllerContinous::compute(){
  if (!_tasks_ptr.size()) _command = mwoibn::VectorN::Zero(_tasks_ptr[0]->getJacobian().cols());
  mwoibn::VectorN q;
  mwoibn::Matrix e;

  mwoibn::Matrix P;
  mwoibn::Matrix J_p;

  P = mwoibn::Matrix::Identity(_dofs,_dofs);
  q = mwoibn::VectorN::Zero(_dofs);
	
    int i = 0;

	for (auto& task:_tasks_ptr){

		//change with respect to the original
		if (_last_stack_change){

      Eigen::ColPivHouseholderQR< mwoibn::Matrix > qr(P); //! \todo check for more efficient implementation, maybe somehow using the SVD that is the most realiable and it is copmuted any way for the JP matrix

			if (qr.rank() != _last_tasks_ranks[i]){	
				_last_tasks_ranks[i] = qr.rank();
				_last_stack_change = 0;
				}
		}

	    e = -(_gains[i].asDiagonal()*task->getError());
		e -=  task->getJacobian()*q;	

		J_p = eigen_utils::agumentedNullSpaceProjection(task->getJacobian(), P, P, 1e-6);//! \todo the accuracy should be an input argument with the default value
 		q += J_p*e;
/*		if (e.norm() && i == 1){
			std::cout << "nr: " << i << ",\t error: \n" << e << "\n";;
			std::cout << "nr: " << i << ",\t J_p: \n" << J_p << "\n";;
			std::cout << "nr: " << i << ",\t J: \n" << task->getJacobian() << "\n";;
		}*/
		// here comes the continuity correction
		++i;
	}
	
	if (!_last_stack_change) _resetCorrection();

    //change with respect to the original
	q += (_g*_e)*exp(-_mu*(_last_stack_change*0.001)); //! \todo ** frequency should be an outside argument

	++_last_stack_change;

  _command = q;
}

void mwoibn::hierarchical_control::HierarchicalControllerContinous::_resetCorrection(){

  std::vector<mwoibn::Matrix> Js_p;
  mwoibn::Matrix P;
  mwoibn::Matrix J_p;

  P = mwoibn::Matrix::Identity(_dofs,_dofs);

	for (auto& task:_tasks_ptr){
		J_p = eigen_utils::agumentedNullSpaceProjection(task->getJacobian(), P, P, 1e-6);//! \todo the accuracy should be an input argument with the default value
		Js_p.push_back(J_p);
	}

  _g = mwoibn::Matrix::Identity(_robot->getDofs(),_robot->getDofs());
	_e.resize(0);
	
  mwoibn::Matrix m;
  mwoibn::VectorN v;

	int i = Js_p.size()-1;
	for (auto it = _tasks_ptr.rbegin(); it != _tasks_ptr.rend(); it++){
		v.resize((*it)->getTaskSize()+_e.size());
		v.head((*it)->getTaskSize()) = _gains[i].asDiagonal()*(*it)->getPreviousError()+(*it)->getPreviousJacobian()*_previous_joint_states;
		v.tail(_e.size()) = _e;		
		_e = v;
		m.resize(_robot->getDofs(),(*it)->getTaskSize()+_g.cols());
		m.rightCols(_g.cols()-_robot->getDofs()) = _g.rightCols(_g.cols()-_robot->getDofs());
    mwoibn::Matrix tm;
		tm.resize(_robot->getDofs(), _robot->getDofs()+(*it)->getTaskSize()); //! \todo check which one is returning the double
    tm.leftCols(_robot->getDofs()) = mwoibn::Matrix::Identity(_robot->getDofs(),_robot->getDofs())-(Js_p[i]*(*it)->getPreviousJacobian());
		tm.rightCols((*it)->getTaskSize()) = Js_p[i];		
		m.leftCols(_robot->getDofs()+(*it)->getTaskSize()) = _g.leftCols(_robot->getDofs())*tm;
		_g = m;
		
		i--;
	}

		_g = _g.rightCols(_g.cols()-_robot->getDofs());

}
