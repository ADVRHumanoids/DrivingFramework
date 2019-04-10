#ifndef __MGNSS_HIGHER_LEVEL_DIFF_H
#define __MGNSS_HIGHER_LEVEL_DIFF_H

#include "mgnss/higher_level/qp/constraints/constraint.h"

namespace mgnss::higher_level::constraints
  {

class Diff: public mgnss::higher_level::Constraint{

  public:
    Diff(Constraint&& constraint, double dt, const mwoibn::VectorN& current ): Constraint(), _dt(dt), _current(current), _constraint(constraint.clone())
    {
      resize(_constraint->getJacobian().rows(), _constraint->getJacobian().cols());
      active.setConstant(_constraint->getJacobian().rows(), true);
    }

    template<typename ConstraintType>
    Diff(std::unique_ptr<ConstraintType> constraint, double dt, const mwoibn::VectorN& current ): Constraint(), _dt(dt), _current(current) {
      _constraint.reset(std::move(constraint));
      resize(_constraint->getJacobian().rows(), _constraint->getJacobian().cols());
      active.setConstant(_constraint->getJacobian().rows(), true);
    }


    Diff(Diff&& other): Constraint(other), _dt(other._dt), _current(other._current){
      _constraint = std::move(other._constraint);
    }

    Diff(const Diff& other): Constraint(other), _constraint(other._constraint->clone()), _dt(other._dt), _current(other._current){ }


    virtual void init() override {
            resize(_constraint->getJacobian().rows(), _constraint->getJacobian().cols());
    }

    virtual void update(){

        _constraint->update();
        _jacobian = _constraint->getJacobian();
        _state = _constraint->getState();
        _state = _state*_dt;

        _state.noalias() += _constraint->getJacobian()*_current;


    }

    mgnss::higher_level::Constraint& constraint(){return *_constraint;}

  protected:
    std::unique_ptr<mgnss::higher_level::Constraint> _constraint;
    double _dt;
    const mwoibn::VectorN& _current;
    //const mwoibn::robot_class::Pipe& _robot_state;
    virtual Diff* clone_impl() const override {return new Diff(*this);}


};

}
#endif
