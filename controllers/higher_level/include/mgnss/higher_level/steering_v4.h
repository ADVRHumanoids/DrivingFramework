#ifndef __MGNSS_HIGHER_LEVEL_STEERING_V4_H
#define __MGNSS_HIGHER_LEVEL_STEERING_V4_H

#include "mgnss/higher_level/steering_reference.h"

namespace mgnss {

namespace higher_level
{

class Steering4 : public SteeringReference
{

public:
Steering4(mwoibn::robot_class::Robot& robot,
          mwoibn::hierarchical_control::tasks::ContactPoint& plane,
          double K_icm, double K_sp, double dt, double margin = 0.04,
          double max = 2.79252680);

virtual ~Steering4() {
}

//virtual void compute(const mwoibn::Vector3 next_step);

protected:

mwoibn::VectorN _temp;
int _pow = 4;

virtual void _ICM(mwoibn::Vector3 next_step);
virtual void _PT(int i);

virtual void _merge(int i);
virtual void _computeTreshhold();
virtual void _resetTreshhold(){
}

double _margin;

};
}
}
#endif // PROGRAM_STEERING_H
