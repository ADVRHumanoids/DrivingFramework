#include "mgnss/higher_level/qr_task.h"


mgnss::higher_level::QrTask::QrTask(int vars, int slack): _vars(vars), _slack(slack), _llt(vars){
            resize(vars, slack);
}

void mgnss::higher_level::QrTask::resize(int vars, int slack){
  _vars = vars;
  _slack = slack;
  _cost.size = vars + slack;

  _optimal_state.setZero(_cost.size); // state + slack

  _cost.quadratic.setIdentity(_cost.size,_cost.size); // this should work only with lower body - how?
  _cost.linear.setZero(_cost.size);
  _return_state.setZero(_vars);

}


void mgnss::higher_level::QrTask::init(){

    _equality.resize(equality.rows(), _optimal_state.size());

    _equality.transpose();

    _inequality.resize( hard_inequality.rows()+2*soft_inequality.rows(), _optimal_state.size());
    _inequality.state.tail(soft_inequality.rows()).setZero();

    _inequality.jacobian.block(hard_inequality.rows(),_vars, soft_inequality.rows(), _slack ) = -mwoibn::Matrix::Identity(soft_inequality.rows(), _slack);
    _inequality.jacobian.block(hard_inequality.rows() + soft_inequality.rows(),_vars, soft_inequality.rows(), _slack ) = -mwoibn::Matrix::Identity(soft_inequality.rows(), _slack);

    // std::cout << "init\n" << _inequality.jacobian << std::endl;
    _inequality.transpose();
}

void mgnss::higher_level::QrTask::clear(){
    equality.clear();
    soft_inequality.clear();
    hard_inequality.clear();
}

void mgnss::higher_level::QrTask::update(){

  if(equality.size()){
    for(auto& constraint: equality) constraint->update();
    _equality.state = equality.getState();
    _equality.jacobian.leftCols(_vars) = equality.getJacobian();

    _equality.transpose();


  }

  if(hard_inequality.size()){
    for(auto& constraint: hard_inequality) constraint->update();
    _inequality.state.head(hard_inequality.rows()) = hard_inequality.getState();
    _inequality.jacobian.block(0,0, hard_inequality.rows(), _vars) = hard_inequality.getJacobian();
  }

  // std::cout << __PRETTY_FUNCTION__  << "\t" << soft_inequality.size() << std::endl;
  //
  // std::cout << "2\t" <<  soft_inequality[0].get().transpose() << std::endl;
  // std::cout << "1\t" << soft_inequality[1].get().transpose() << std::endl;
  if(soft_inequality.size()){
    for(auto& constraint: soft_inequality) constraint->update();

    _inequality.state.segment(hard_inequality.rows(), soft_inequality.rows()) = soft_inequality.getState();
    _inequality.jacobian.block(hard_inequality.rows(),0, soft_inequality.rows(), _vars) =  soft_inequality.getJacobian();
  }
    _inequality.transpose();

}

void mgnss::higher_level::QrTask::solve(){


    update();
    //
     // std::cout << "_cost.quadratic\n" << _cost.quadratic << std::endl;
     // std::cout << "_cost.linear\n" << _cost.linear.transpose() << std::endl;
     // std::cout << "inequality.state\t" << _inequality.state.transpose() << std::endl;
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

    cost__ = solve_quadprog2(_llt, _trace, _cost.linear, _equality.transposed, _equality.state, _inequality.transposed, _inequality.state, _optimal_state);
    std::cout << "cost__\t" << cost__ << std::endl;
    _return_state = _optimal_state.head(_vars);
    _outputTransform();
    // if(_return_state.norm()) std::cout << "_optimal_state\t" << _return_state.transpose() << std::endl;
    // std::cout << "return state\t" << _return_state.transpose() << std::endl;


 //    std::cout << "soft_inequality.state\n" << soft_inequality.getState().transpose() << std::endl;
 //    std::cout << "_inequality.transposed\n" << _inequality.transposed << std::endl;
 //    std::cout << "_inequality.state\n" << _inequality.state.transpose() << std::endl;
 // std::cout << "_cost.quadratic\n" << _cost.quadratic << std::endl;
}
