#ifndef __MWOIBN_ROBOT_CLASS_STEERING_H
#define __MWOIBN_ROBOT_CLASS_STEERING_H

#include "mwoibn/robot_class/angles/basic.h"

namespace mwoibn
{
namespace robot_class
{
namespace angles {
/**
 * @brief The CartesianWorld class Provides the inverse kinematics task
 *******************to control the position of a point defined in one of a robot reference frames
 *
 */

class Steering : public Basic
{
public:
Steering(mwoibn::robot_class::Robot& robot,
         mwoibn::point_handling::Frame point,
         mwoibn::Axis axis)
        : Basic(robot, point, axis)
{
        _x_world << 1, 0, 0;
        _y_world << 0, 1, 0;
        _z_world << 0, 0, 1;
        _J.setZero(1, robot.getDofs());

}   // for now just support major axes

virtual ~Steering() {
}

virtual void update()
{
        _v1 = _x_world;

        _n = _z_world;

        _y = _point.getRotationWorld() * _axis;
        _v2 = _y.cross(_z_world);

        mwoibn::Scalar b = 1 / _v2.norm();

        _v2.normalize();

        mwoibn::Scalar cross = (_v1.cross(_v2)).transpose() * _n;
        mwoibn::Scalar dot = _v1.transpose() * _v2;

        _angle = std::atan2(cross, dot);

        // DERIVATIVE

        mwoibn::Scalar O = cross * cross + dot * dot;

        mwoibn::Scalar P = dot / O;
        O = -cross / O;

        mwoibn::eigen_utils::skew(_y, _s_y);
        mwoibn::eigen_utils::skew(_n, _s_n);
        mwoibn::eigen_utils::skew(_v1, _s_v1);

        _mB = _y * _n.transpose();
        _vA = _mB * _y;

        mwoibn::eigen_utils::skew(_vA, _mA);

        _mA += _mB * _s_y; // _z_body or world?
        _tA = -0.5 * b * b * b * _n.transpose() * _mA;

        _mB = _s_y * _n * _tA;
        _mB += b * _s_n * _s_y;

        _tA = _v1.transpose() * _mB;
        _tB = _n.transpose() * _s_v1 * _mB;

        _tA = O * _tA;
        _tA += P * _tB;

        _J.noalias() = _tA * _point.getOrientationJacobian();
}

protected:
mwoibn::Axis _v2, _n, _v1, _y;
mwoibn::Axis _x_world, _y_world,
             _z_world; // for now assume the basic initialization

mwoibn::Scalar _d_steering;

mwoibn::Vector3 _vA;
mwoibn::Vector3T _tA, _tB;

mwoibn::Matrix3 _mA, _mB, _s_y, _s_n, _s_v1;
virtual Steering* clone_impl() const {return new Steering(*this);}

};

}
} // namespace package
} // namespace library
#endif
