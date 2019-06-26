#include "mgnss/higher_level/qp/constraints/merge.h"

void mgnss::higher_level::constraints::Merge::update(){
    for(auto& constraint: _constraints) constraint->update();

    _state.setConstant(mwoibn::MAX_DOUBLE); // deactivate all
    _jacobian.setZero();

    for(int i = 0, k = 0; i < active.size(); i++){
      // std::cout << i << ", " << k << ":";
      if (!active[i]) continue;
      _jacobian.row(k) = _constraints[0].getJacobian().row(i);
      // std::cout << "\tnew:\t";

      for (auto& constraint: _constraints){
              if(constraint->active[i]){
                _state[k] = std::min(_state[k], constraint->getState()[i]);
                 // std::cout << " " << constraint->getState()[i];
                }
            //Do I need it?

            _state[k] = std::max(_state[k], -0.1);
      }
      // std::cout << "sol\t" << _state[k] << std::endl;
//      std::cout << std::endl;
      k++;
    }
}


    void mgnss::higher_level::constraints::Merge::_checkConstraint(){
      if(_constraints.size() == 1)
        resize(_constraints[0].rows(), _constraints[0].cols());
      else{
        if (_constraints[0].rows() != _constraints.end(0).rows())
        throw std::invalid_argument(__PRETTY_FUNCTION__ +
            std::string( "unconsistent constraint state, expected " +
            std::to_string(_constraints[0].rows()) + " received " +
            std::to_string(_constraints.end(0).rows()) ) );

        if (_constraints[0].rows() != _constraints.end(0).rows())
            throw std::invalid_argument(__PRETTY_FUNCTION__ +
                std::string( "unconsistent constraint variables, expected " +
                std::to_string(_constraints[0].cols()) + " received " +
                std::to_string(_constraints.end(0).cols()) ) );

        if (_constraints[0].getJacobian() != _constraints.end(0).getJacobian())
            throw std::invalid_argument(__PRETTY_FUNCTION__ +
              std::string( ": Could not initialize merge constraint expected consistent constraint jacobians") );
      }
    }
