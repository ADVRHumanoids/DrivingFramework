#ifndef REFERENCE_GENERATION_LINE_H
#define REFERENCE_GENERATION_LINE_H

#include "mwoibn/reference_generation/reference_generation.h"
#include "mwoibn/reference_generation/utils.h"
#include <Eigen/Dense>
#include "mwoibn/reference_generation/basic_object.h"


namespace mwoibn{
namespace reference_generation
{

inline Eigen::VectorXd makeLine(const Eigen::VectorXd& x, const Eigen::VectorXd& x_des,
                         double x_max)
{

  Eigen::VectorXd d = x_des - x;
  if (d.norm() > x_max)
    d = x + x_max * d.normalized();
  else
    d = x_des;

  return d;
}

class Line: public BasicObject<Eigen::VectorXd>
{
public:
  Line(Eigen::VectorXd origin_state, double step,
       Eigen::VectorXd final_state)
      : BasicObject()
  {
     _step = step;
     _current_state = origin_state;
     _origin_state = origin_state;
    _final_state = final_state;
  }
  virtual ~Line() {}

  virtual Eigen::VectorXd nextStep();

  virtual Eigen::VectorXd backStep();

  virtual Eigen::VectorXd getFinalPoint() {return _final_state;}

//  virtual void setFinalPoint(Eigen::VectorXd new_final_state) {_final_state = new_final_state;}

  virtual Eigen::VectorXd getOriginPoint() {return _origin_state;}

  virtual Eigen::VectorXd getCurrentPoint() {return _current_state;}

};

} // namespace package
} // namespace library
#endif // REFERENCE_GENERATION_LINE_H
