#ifndef __MGNSS_HIGHER_LEVEL_SUPPORT_CONSTRAINT_H
#define __MGNSS_HIGHER_LEVEL_SUPPORT_CONSTRAINT_H

#include "eiquadprog/eiquadprog.hh" //?
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
    Constraint(){    }
    Constraint(int size, int vars){
      resize(size, vars);
    }

    void resize(int size, int vars){
      if(vars == 0) size = 0;

      jacobian.setZero(size, vars);
      transposed.setZero(vars, size);
      state.setZero(size);
    }

    const mwoibn::VectorN& get(){return state;}
    const mwoibn::Matrix& getJacobian(){return jacobian;}
    int rows(){return state.size();}
    int cols(){return jacobian.cols();}


    void transpose(){ transposed = jacobian.transpose();}

    mwoibn::Matrix jacobian, transposed;
    mwoibn::VectorN state;

    int size(){return state.size() ;}

    virtual void update(){}

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
