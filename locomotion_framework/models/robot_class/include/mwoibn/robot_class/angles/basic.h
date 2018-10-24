#ifndef __MWOIBN_ROBOT_CLASS_ANGLES_BASIC_H
#define __MWOIBN_ROBOT_CLASS_ANGLES_BASIC_H

#include "mwoibn/point_handling/robot_points_handler.h"
#include <rbdl/rbdl.h>

namespace mwoibn
{
namespace robot_class
{
namespace angles
{
/**
 * @brief The CartesianWorld class Provides the inverse kinematics task
 * to control the position of a point defined in one of a robot reference frames
 *
 */

class Basic
{
public:
Basic(mwoibn::robot_class::Robot& robot,
      mwoibn::point_handling::Frame point,
      mwoibn::Axis axis)
        : _axis(axis), _point(point), _robot(robot){
}    // for now just support major axes

virtual ~Basic() {
}

virtual void update() = 0;

mwoibn::Scalar get(){
        return _angle;
}
const mwoibn::Matrix& getJacobian(){
        return _J;
}

protected:
mwoibn::Axis _axis;

double _angle;
mwoibn::point_handling::Frame _point;
mwoibn::robot_class::Robot& _robot;

mwoibn::Matrix _J;


};
}
} // namespace package
} // namespace library

#endif
