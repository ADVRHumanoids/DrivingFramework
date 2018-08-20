#ifndef __MGNSS_HIGHER_LEVEL_STEERING_V5_H
#define __MGNSS_HIGHER_LEVEL_STEERING_V5_H

#include "mgnss/higher_level/steering_reference.h"

namespace mgnss
{

namespace higher_level
{

class Steering5 : public SteeringReference
{

public:
Steering5(mwoibn::robot_class::Robot& robot,
          mwoibn::hierarchical_control::tasks::ContactPointTracking& plane,
          mwoibn::VectorN init_pose, double K_icm, double K_sp, double dt,
          double margin = 0.04, double max = 2.79252680);

virtual ~Steering5() {
}

protected:

virtual void _merge(int i);
mwoibn::VectorN _pb_icm, _pb_sp;

virtual void _ICM(mwoibn::Vector3 next_step);

virtual void _PT(int i);

virtual void _computeTreshhold();
virtual void _resetTreshhold();


};
}
}
#endif // PROGRAM_STEERING_H
