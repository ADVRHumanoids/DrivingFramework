#ifndef PROGRAM_STEERING_V4_H
#define PROGRAM_STEERING_V4_H

#include "mgnss/controllers/steering_reference.h"

namespace mgnss {

namespace events
{

class Steering4 : public SteeringReference
{

public:
Steering4(mwoibn::robot_class::Robot& robot,
          mwoibn::hierarchical_control::tasks::ContactPointTracking& plane, mwoibn::VectorN init_pose,
          double K_icm, double K_sp, double dt, double margin = 0.04,
          double max = 2.79252680);

~Steering4() {
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
