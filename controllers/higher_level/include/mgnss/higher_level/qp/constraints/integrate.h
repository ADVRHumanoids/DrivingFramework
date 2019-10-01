#ifndef __MGNSS_HIGHER_LEVEL_INTEGRATE_H
#define __MGNSS_HIGHER_LEVEL_INTEGRATE_H

#include "mgnss/higher_level/qp/constraints/constraint.h"

namespace mgnss::higher_level::constraints
  {

class Integrate: public mgnss::higher_level::Constraint{

  public:
    // template<typename ConstraintType>
    // Integrate(ConstraintType constraint, double dt, const mwoibn::robot_class::Pipe& robot_state ): Constraint(constraint), _constraint(constraint), _dt(dt), _robot_state(robot_state) {
    //   _constraint.reset(new ConstraintType(constraint));
    // }

    //template<typename ConstraintType>
    Integrate(Constraint&& constraint, double dt, const mwoibn::VectorN& current, bool positive_sign = true): Constraint(), _dt(dt), _current(current), _constraint(constraint.clone())
    {
      _sign = (positive_sign) ? 1 : -1;
      resize(_constraint->getJacobian().rows(), _constraint->getJacobian().cols());
      active.setConstant(_constraint->getJacobian().rows(), true);
    }

    template<typename ConstraintType>
    Integrate(std::unique_ptr<ConstraintType> constraint, double dt, const mwoibn::VectorN& current, bool positive_sign = true ): Constraint(), _dt(dt), _current(current) {
      _sign = (positive_sign) ? 1 : -1;
      _constraint.reset(std::move(constraint));
      resize(_constraint->getJacobian().rows(), _constraint->getJacobian().cols());
      active.setConstant(_constraint->getJacobian().rows(), true);
    }


    Integrate(Integrate&& other): Constraint(other), _dt(other._dt), _current(other._current), _sign(other._sign){
      _constraint = std::move(other._constraint);
    }

    Integrate(const Integrate& other): Constraint(other), _constraint(other._constraint->clone()), _dt(other._dt), _current(other._current), _sign(other._sign){ }


    virtual void init() override {
            resize(_constraint->getJacobian().rows(), _constraint->getJacobian().cols());
    }

    virtual void update(){
      // std::cout << "Constraint" << std::endl;
        _constraint->update();
        _jacobian = _constraint->getJacobian();
        _state = _constraint->getState();
        _state.noalias() += _sign*_current;



        _state = _state/_dt;
        // std::cout << "_constraint->state\t_state\t_current\t" << _sign << std::endl;
        // for(int i = 0; i < active.size(); i++)
          // std::cout << _constraint->getState()[i] << "\t" << _state[i] << "\t" << _current[i] << "\n";

        //std::cout << "current\t" << _current.transpose() << std::endl;
        // std::cout << "jacobian\n" << jacobian << std::endl;

    }

    mgnss::higher_level::Constraint& constraint(){return *_constraint;}

  protected:
    std::unique_ptr<mgnss::higher_level::Constraint> _constraint;
    double _dt;
    const mwoibn::VectorN& _current;
    //const mwoibn::robot_class::Pipe& _robot_state;
    virtual Integrate* clone_impl() const override {return new Integrate(*this);}
    int _sign;

};

}
#endif
