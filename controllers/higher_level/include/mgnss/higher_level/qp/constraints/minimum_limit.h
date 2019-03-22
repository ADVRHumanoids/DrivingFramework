#ifndef __MGNSS_HIGHER_LEVEL_MINIMUM_LIMIT_H
#define __MGNSS_HIGHER_LEVEL_MINIMUM_LIMIT_H

#include "mgnss/higher_level/qp/constraints/constraint.h"


namespace mgnss
{

namespace higher_level
{

  namespace constraints
  {


class MinimumLimit: public mgnss::higher_level::Constraint{

  public:
    MinimumLimit(const mwoibn::Matrix& jacobian, double limit ): Constraint(jacobian.rows(), jacobian.cols()), _other_jacobian(jacobian){
        _state = -mwoibn::VectorN::Constant(size(), limit);
    }


    virtual void update(){
        _jacobian = _other_jacobian;
    }

  protected:
    const mwoibn::Matrix& _other_jacobian;
    virtual MinimumLimit* clone_impl() const override {return new MinimumLimit(*this);}

};

}
}
}
#endif
