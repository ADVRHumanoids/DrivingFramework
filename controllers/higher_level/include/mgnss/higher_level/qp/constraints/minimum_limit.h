#ifndef __MGNSS_HIGHER_LEVEL_MINIMUM_LIMIT_H
#define __MGNSS_HIGHER_LEVEL_MINIMUM_LIMIT_H

#include "mgnss/higher_level/qp/constraints/constraint.h"


namespace mgnss::higher_level::constraints
  {

// It support only constant limits
class MinimumLimit: public mgnss::higher_level::Constraint{

  public:
    // MinimumLimit(const mwoibn::Matrix& jacobian, double limit ): Constraint(jacobian.rows(), jacobian.cols()), _other_jacobian(jacobian){
    //     _state = -mwoibn::VectorN::Constant(size(), limit);
    // }

    MinimumLimit(const mwoibn::Matrix& jacobian, const mwoibn::VectorN& limits ): Constraint(jacobian.rows(), jacobian.cols()), _other_jacobian(jacobian), _limits(limits){
        _state = -limits;
    }

    MinimumLimit(const MinimumLimit& other): Constraint(other), _other_jacobian(other._other_jacobian), _limits(other._limits){
    }


    virtual void update(){
        _state = -_limits;
        _jacobian = _other_jacobian;
    }

  protected:
    const mwoibn::Matrix& _other_jacobian;
    virtual MinimumLimit* clone_impl() const override {return new MinimumLimit(*this);}
    const mwoibn::VectorN& _limits;
};
}
#endif
