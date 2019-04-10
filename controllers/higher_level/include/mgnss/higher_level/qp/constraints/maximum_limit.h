#ifndef __MGNSS__HIGHER_LEVEL__MAXIMUM_LIMIT_H
#define __MGNSS__HIGHER_LEVEL__MAXIMUM_LIMIT_H

#include "mgnss/higher_level/qp/constraints/constraint.h"

namespace mgnss::higher_level::constraints
  {

class MaximumLimit: public mgnss::higher_level::Constraint{

  public:
    // MaximumLimit(const mwoibn::Matrix& jacobian, double limit ): mgnss::higher_level::Constraint(jacobian.rows(), jacobian.cols()), _other_jacobian(jacobian){
    //     _state = mwoibn::VectorN::Constant(size(), limit);
    // }

    MaximumLimit(const mwoibn::Matrix& jacobian, const mwoibn::VectorN& limits ): mgnss::higher_level::Constraint(jacobian.rows(), jacobian.cols()), _other_jacobian(jacobian), _limits(limits){
        _state = _limits;
    }

    MaximumLimit(const MaximumLimit& other): mgnss::higher_level::Constraint(other), _other_jacobian(other._other_jacobian), _limits(other._limits){
    }

    virtual void update(){
      _state = _limits;
       _jacobian = -_other_jacobian;
    }

  protected:
    virtual MaximumLimit* clone_impl() const override {return new MaximumLimit(*this);}
    const mwoibn::Matrix& _other_jacobian;
    const mwoibn::VectorN& _limits;



};

}
#endif
