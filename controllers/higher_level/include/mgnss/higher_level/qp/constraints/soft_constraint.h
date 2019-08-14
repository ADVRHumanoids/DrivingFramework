#ifndef __MGNSS__HIGHER_LEVEL__SOFT_CONSTRAINT_H
#define __MGNSS__HIGHER_LEVEL__SOFT_CONSTRAINT_H

#include "mgnss/higher_level/qp/constraints/soft_constraint.h"


namespace mgnss::higher_level::constraints
{


class SoftConstraint: public Constraint{

  public:
    template<typename ConstraintType>
    SoftConstraint(ConstraintType&& constraint, double gain): Constraint() {
      _constraint.reset(new ConstraintType(std::move(constraint)));
      _gain.setConstant(_constraint->rows(), gain);
      resize(_constraint->getJacobian().rows(), _constraint->getJacobian().cols());
    }

    template<typename ConstraintType>
    SoftConstraint(std::unique_ptr<ConstraintType> constraint, double gain ): Constraint(), _gain(gain) {
      _constraint.reset(std::move(constraint));
      _gain.setConstant(_constraint->rows(), gain);
      resize(_constraint->getJacobian().rows(), _constraint->getJacobian().cols());
    }
    template<typename ConstraintType>
    SoftConstraint(ConstraintType&& constraint, const mwoibn::VectorN& gain): Constraint(), _gain(gain) {
      _constraint.reset(new ConstraintType(constraint));
      resize(_constraint->getJacobian().rows(), _constraint->getJacobian().cols());
    }

    template<typename ConstraintType>
    SoftConstraint(std::unique_ptr<ConstraintType> constraint, const mwoibn::VectorN&  gain ): Constraint(), _gain(gain) {
      _constraint.reset(std::move(constraint));
      resize(_constraint->getJacobian().rows(), _constraint->getJacobian().cols());
    }

    SoftConstraint(SoftConstraint&& other): Constraint(other), _gain(other._gain){
      _constraint = std::move(other._constraint);
    }

    SoftConstraint(const SoftConstraint& other): Constraint(other), _constraint(other._constraint->clone()), _gain(other._gain){ }


    virtual void init(){_constraint->init();}

    auto clone() {return std::unique_ptr<SoftConstraint>(clone_impl()); }

    const mwoibn::VectorN& getGain(){return _gain;}

    void setGain(const mwoibn::VectorN& gain){_gain = gain;}


    virtual const mwoibn::Matrix& getJacobian() const {return _constraint->getJacobian();}
    virtual mwoibn::Matrix& setJacobian(){return _constraint->setJacobian();}

    virtual const mwoibn::Matrix& getTransposed() const {return _constraint->getTransposed();}
    virtual mwoibn::Matrix& setTransposed(){return _constraint->setTransposed();}

    virtual const mwoibn::VectorN& getState() const {return _constraint->getState();}
    virtual mwoibn::VectorN& setState(){return _constraint->setState();}


    virtual void update(){
      _constraint->update();
      _jacobian = _constraint->getJacobian();//this is not needed
      _state = _constraint->getState();
    }


  protected:
    std::unique_ptr<mgnss::higher_level::Constraint> _constraint;
    mwoibn::VectorN _gain;
    virtual SoftConstraint* clone_impl() const override {return new SoftConstraint(*this);}

};



}
#endif
