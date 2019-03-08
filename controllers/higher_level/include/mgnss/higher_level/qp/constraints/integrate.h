#ifndef __MGNSS_HIGHER_LEVEL_INTEGRATE_H
#define __MGNSS_HIGHER_LEVEL_INTEGRATE_H

#include "mgnss/higher_level/qp/constraints/constraint.h"

namespace mgnss
{

namespace higher_level
{

  namespace constraints
  {

class Intergate: public mgnss::higher_level::Constraint{

  public:
    // template<typename ConstraintType>
    // Intergate(ConstraintType constraint, double dt, const mwoibn::robot_class::Pipe& robot_state ): Constraint(constraint), _constraint(constraint), _dt(dt), _robot_state(robot_state) {
    //   _constraint.reset(new ConstraintType(constraint));
    // }

    template<typename ConstraintType>
    Intergate(ConstraintType&& constraint, double dt, const mwoibn::robot_class::Pipe& robot_state ): Constraint(), _dt(dt), _robot_state(robot_state) {
      _constraint.reset(new ConstraintType(constraint));
      resize(_constraint->jacobian.rows(), _constraint->jacobian.cols());
    }

    template<typename ConstraintType>
    Intergate(std::unique_ptr<ConstraintType> constraint, double dt, const mwoibn::robot_class::Pipe& robot_state ): Constraint(), _dt(dt), _robot_state(robot_state) {
      _constraint.reset(std::move(constraint));
      resize(_constraint->jacobian.rows(), _constraint->jacobian.cols());
    }


    Intergate(Intergate&& other): Constraint(other), _dt(other._dt), _robot_state(other._robot_state){
      _constraint = std::move(other._constraint);
    }

    Intergate(const Intergate& other): Constraint(other), _constraint(other._constraint->clone()), _dt(other._dt), _robot_state(other._robot_state){ }


    virtual void update(){
      _constraint->update();
        jacobian = _constraint->jacobian*_dt;
        state = _constraint->state;
        state += _constraint->jacobian*_robot_state.get();

        std::cout << "Constraint\n" << std::endl;
        std::cout << "_constraint->state\t" << _constraint->state.transpose() << std::endl;
        std::cout << "_constraint->jacobian\n" << _constraint->jacobian << std::endl;
        std::cout << "state\t" << state.transpose() << std::endl;
        std::cout << "jacobian\n" << jacobian << std::endl;

    }

  protected:
    std::unique_ptr<mgnss::higher_level::Constraint> _constraint;
    double _dt;
    const mwoibn::robot_class::Pipe& _robot_state;
    virtual Intergate* clone_impl() const override {return new Intergate(*this);}


};

}
}
}
#endif
