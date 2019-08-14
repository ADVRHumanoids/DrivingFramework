#ifndef __MGNSS__HIGHER_LEVEL__CONSTRAINT_H
#define __MGNSS__HIGHER_LEVEL__CONSTRAINT_H

//#include "eiquadprog/eiquadprog.hh" //?
#include "mwoibn/robot_class/robot.h"
#include "mwoibn/hierarchical_control/tasks/controller_task.h"
#include "mwoibn/robot_points/handler.h"
#include "mwoibn/common/logger.h"


namespace mgnss
{

namespace higher_level
{


class Constraint{

  public:
    Constraint(){   resize(0,0); }
    Constraint(int size, int vars){
      resize(size, vars);
      active_dofs = mwoibn::eigen_utils::iota(vars);
      active.setConstant(size, true);
    }

    Constraint(int size, int vars, mwoibn::VectorInt& dofs): active_dofs(dofs){
      resize(size, vars);
      active.setConstant(size, true);
    }

    Constraint(const Constraint& other): _jacobian(other._jacobian),
          _transposed(other._transposed), _state(other._state), active_dofs(other.active_dofs), active(other.active){
          }

    virtual void init(){}

    auto clone() {return std::unique_ptr<Constraint>(clone_impl()); }

    void resize(int size, int vars, bool reset_active = true){
      if(vars == 0) size = 0;

      _jacobian.setZero(size, vars);
      _transposed.setZero(vars, size);
      _state.setZero(size);
      if(!reset_active) return;
      active_dofs = mwoibn::eigen_utils::iota(vars);
      active.setConstant(size, true);
    }

    const mwoibn::VectorN& get(){return _state;}
    int rows(){return _state.size();}
    int cols(){return _jacobian.cols();}


    void transpose(){ _transposed = _jacobian.transpose();}



    int size(){return _state.size() ;}

    virtual void update(){}

    virtual const mwoibn::Matrix& getJacobian() const {return _jacobian;}
    virtual mwoibn::Matrix& setJacobian(){return _jacobian;}

    virtual const mwoibn::Matrix& getTransposed() const {return _transposed;}
    virtual mwoibn::Matrix& setTransposed(){return _transposed;}

    virtual const mwoibn::VectorN& getState() const {return _state;}
    virtual mwoibn::VectorN& setState(){return _state;}

    mwoibn::VectorInt active_dofs;
    mwoibn::VectorBool active;

  protected:
    virtual Constraint* clone_impl() const {return new Constraint(*this);}
    mwoibn::Matrix _jacobian, _transposed;
    mwoibn::VectorN _state;

};

struct Cost{
  mwoibn::Matrix quadratic;
  mwoibn::VectorN linear;
  double trace;
  int size;
};

struct Limit: public Constraint {

  mwoibn::VectorN limit;
  // mwoibn::VectorN state;
  // mwoibn::Matrix jacobian;
  mwoibn::VectorN error;
};

}
}
#endif
