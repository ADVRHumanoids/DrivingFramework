#include "mwoibn/reference_generation/line.h"


  Eigen::VectorXd mwoibn::reference_generation::Line::nextStep()
  {
    Eigen::VectorXd d = _final_state - _current_state;
    if (d.norm() > _step)
      _current_state += _step * d.normalized();
    else
      _current_state = _final_state;

    return _current_state;
  }

  Eigen::VectorXd mwoibn::reference_generation::Line::backStep()
  {
    Eigen::VectorXd d = _final_state - _current_state;

    _current_state -= _step * d.normalized();

    return _current_state;
  }


