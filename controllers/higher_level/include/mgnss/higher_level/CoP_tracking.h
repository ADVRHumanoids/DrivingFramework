#ifndef __MGNSS_HIGHER_LEVEL_COP_TRACKING_H
#define __MGNSS_HIGHER_LEVEL_COP_TRACKING_H

#include <mwoibn/robot_class/robot.h>

namespace mgnss {

namespace higher_level
{


// I need
//    CoM task error
//    CoM_{des}
//    CoM task gain,

// current
//    \dot CoM

//    desired distance,
//    ground normal
class CopTracking
{

public:
CopTracking(mwoibn::robot_class::Robot& robot)
{

}

virtual ~CopTracking() {
}

protected:


};
}
}
#endif // PROGRAM_STEERING_H
