#include "mgnss/higher_level/qp/tasks/qr_task.h"





mgnss::higher_level::QrTask::QrTask(int vars, int slack): _vars(vars), _slack(slack), _llt(vars){
            resize(vars, slack);

}

mgnss::higher_level::QrTask::QrTask(): _vars(0), _slack(0), _llt(0){
            resize(0, 0);
}

void mgnss::higher_level::QrTask::resize(int vars, int slack){
  _vars = vars;
  _slack = slack;
  _cost.size = vars + slack;

  _optimal_state.setZero(_cost.size); // state + slack
  _soft_gains.setZero(slack);
  _updateGains();

  _cost.quadratic.setIdentity(_cost.size,_cost.size); // this should work only with lower body - how?
  _cost.linear.setZero(_cost.size);
  _return_state.setZero(_vars);
  _init(_vars , _vars);
}


void mgnss::higher_level::QrTask::init(){

    _equality.resize(equality.rows(), _optimal_state.size());

    _equality.transpose();

    _inequality.resize( hard_inequality.rows()+2*soft_inequality.rows(), _optimal_state.size());
    _inequality.setState().tail(soft_inequality.rows()).setZero();

    _inequality.setJacobian().block(hard_inequality.rows(),_vars, soft_inequality.rows(), _slack ) = -mwoibn::Matrix::Identity(soft_inequality.rows(), _slack);
    _inequality.setJacobian().block(hard_inequality.rows() + soft_inequality.rows(),_vars, soft_inequality.rows(), _slack ) = -mwoibn::Matrix::Identity(soft_inequality.rows(), _slack);

    _inequality.transpose();

    _solver.init(  _cost.size, _equality.rows(), _inequality.rows());
}

void mgnss::higher_level::QrTask::clear(){
    equality.clear();
    soft_inequality.clear();
    hard_inequality.clear();
}

void mgnss::higher_level::QrTask::_update(){

  if(equality.size()){
    for(auto& constraint: equality) {
      constraint->update();
    }
    _equality.setState() = equality.getState();

    for(int i = 0; i < _equality.active_dofs.size(); i++ )
      _equality.setJacobian().col(i) = equality.getJacobian().col(_equality.active_dofs[i]);

    // std::cout << _equality.getState().transpose() << std::endl;
    _equality.transpose();


  }

  if(hard_inequality.size()){
    for(auto& constraint: hard_inequality) constraint->update();
    _inequality.setState().head(hard_inequality.rows()) = hard_inequality.getState();

    for(int i = 0; i < _inequality.active_dofs.size(); i++)
      _inequality.setJacobian().block(0, i, hard_inequality.rows(), 1) = hard_inequality.getJacobian().col(_inequality.active_dofs[i]);

    // _inequality.setJacobian().block(0,0, hard_inequality.rows(), _vars) = hard_inequality.getJacobian();
  }


  if(soft_inequality.size()){
    for(auto& constraint: soft_inequality) {
      constraint->update();
    }

    _inequality.setState().segment(hard_inequality.rows(), soft_inequality.rows()) = soft_inequality.getState();
    for(int i = 0; i < _inequality.active_dofs.size(); i++)
        _inequality.setJacobian().block(hard_inequality.rows(),i, soft_inequality.rows(), 1) =  soft_inequality.getJacobian().col(_inequality.active_dofs[i]);

    // _inequality.setJacobian().block(hard_inequality.rows(),0, soft_inequality.rows(), _vars) =  soft_inequality.getJacobian();
  }
    _inequality.transpose();

    _updateGains();
    _cost.quadratic.block(_vars, _vars, _slack, _slack) = _soft_gains.asDiagonal();
}

void mgnss::higher_level::QrTask::solve(){


    _update();
    //
     // std::cout << "_cost.quadratic\n" << _cost.quadratic << std::endl;
     // std::cout << "_cost.linear\n" << _cost.linear.transpose() << std::endl;
     // std::cout << "soft_inequality.state\t" << soft_inequality.getState().transpose() << std::endl;
     // std::cout << "hard_inequality.state\t" << hard_inequality.getState().transpose() << std::endl;
     // std::cout << "inequality.state\t" << _inequality.getState().transpose() << std::endl;
     // std::cout << "inequality.jacobian\t" << _inequality.jacobian << std::endl;
     // std::cout << "inequality.jacobian\t" << _inequality.jacobian << std::endl;
     // std::cout << "initial conditions\t" << _optimal_state.transpose() << std::endl;
    //

     // std::cout << "soft_inequality\t" << soft_inequality.getJacobian() << std::endl;
     // std::cout << "state\t" << soft_inequality.getState().transpose() << std::endl;
     // std::cout << "_equality.transposed\n" << _equality.transposed << std::endl;
     // std::cout << "_equality.state\n" << _equality.state.transpose() << std::endl;



    _llt.compute(_cost.quadratic);
    _cost.trace = _cost.quadratic.trace();

    _optimal_cost = _solver.solve_quadprog2(_llt, _trace, _cost.linear, _equality.getTransposed(), _equality.getState(), _inequality.getTransposed(), _inequality.getState(), _optimal_state);
    std::cout << "_optimal_cost\t" << _optimal_cost << std::endl;
    _return_state = _optimal_state.head(_vars);
    _outputTransform();
    // if(_return_state.norm()) std::cout << "_optimal_state\t" << _return_state.transpose() << std::endl;
    // std::cout << "return state\t" << _return_state.transpose() << std::endl;


 //    std::cout << "soft_inequality.state\n" << soft_inequality.getState().transpose() << std::endl;
 //    std::cout << "_inequality.transposed\n" << _inequality.transposed << std::endl;
 //    std::cout << "_inequality.state\n" << _inequality.state.transpose() << std::endl;
 // std::cout << "_cost.quadratic\n" << _cost.quadratic << std::endl;
}
