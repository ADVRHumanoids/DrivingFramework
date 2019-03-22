#ifndef __MGNSS__HIGHER_LEVEL__MAXIMUM_LIMIT_H
#define __MGNSS__HIGHER_LEVEL__MAXIMUM_LIMIT_H

#include "mgnss/higher_level/qp/constraints/constraint.h"

namespace mgnss
{

namespace higher_level
{

  namespace constraints
  {

class MaximumLimit: public mgnss::higher_level::Constraint{

  public:
    MaximumLimit(const mwoibn::Matrix& jacobian, double limit ): mgnss::higher_level::Constraint(jacobian.rows(), jacobian.cols()), _other_jacobian(jacobian){
        _state = mwoibn::VectorN::Constant(size(), limit);
    }

    MaximumLimit(const MaximumLimit& other): mgnss::higher_level::Constraint(other), _other_jacobian(other._other_jacobian){
    }

    virtual void update(){
       _jacobian = -_other_jacobian;
    }

  protected:
    virtual MaximumLimit* clone_impl() const override {return new MaximumLimit(*this);}
    const mwoibn::Matrix& _other_jacobian;



};

}
}
}
#endif
