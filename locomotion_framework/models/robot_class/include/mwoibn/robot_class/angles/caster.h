#ifndef __MWOIBN_ROBOT_CLASS_CASTER_H
#define __MWOIBN_ROBOT_CLASS_CASTER_H

#include "mwoibn/robot_class/angles/basic.h"

namespace mwoibn
{
namespace robot_class
{
namespace angles {

/**
 * @brief The CartesianWorld class Provides the inverse kinematics task
 * to control the position of a point defined in one of a robot reference frames
 *
 */

class Caster : public Basic
{
public:
Caster(mwoibn::robot_class::Robot& robot,
       mwoibn::point_handling::Point point, mwoibn::Axis axis)
        : Basic(robot, point, axis)
{
        _x_world << 1, 0, 0;
        _y_world << 0, 1, 0;
        _z_world << 0, 0, 1;
        _J.setZero(1, robot.getDofs());

}   // for now just support major axes

~Caster() {
}

virtual void update()
{
        // axis - z
        _v1 = _point.getRotationWorld(_robot.state.get()) * _z_world;

        _n = _point.getRotationWorld(_robot.state.get()) * _axis;

        _v2 = _z_world - _n * _z_world.transpose() * _n;

        double b = 1 / _v2.norm();
        _v2.normalize();

        double cross = (_v1.cross(_v2)).transpose() * _n;
        double dot = _v1.transpose() * _v2;
        _angle = std::atan2(cross, dot);

        // DERIVATIVE

        double A = cross * cross + dot * dot;

        double B = dot / A;
        A = -cross / A;

        mwoibn::eigen_utils::skew(_v1, _s_v1);
        mwoibn::eigen_utils::skew(_v2, _s_v2);
        mwoibn::eigen_utils::skew(_n, _s_n);

        _mB = _n * _z_world.transpose();
        _vA = _mB * _n;

        mwoibn::eigen_utils::skew(_vA, _mA);

        _mA += _mB * _s_n;

        _mB = 0.5 * b * b * _z_world * _z_world.transpose();
        _mB -= mwoibn::Matrix3::Identity();
        _mB -= 0.5 * b * b * _n * _n.transpose() * _z_world * _z_world.transpose();
        _mB = _mB * b;

        _mC = -_mB * _mA;

        _tA = _v1.transpose() * _mC;
        _tA -= _v2.transpose() * _s_v1;

        _tB = _n.transpose() * _s_v2 * _s_v1;
        _tB += _n.transpose() * _s_v1 * _mC;
        _tB -= (_s_v1 * _v2).transpose() * _s_n;

        _tA = A * _tA;
        _tA += B * _tB;

        _J.noalias() = _tA * _point.getOrientationJacobian(_robot.state.get());
}

protected:
mwoibn::Axis _v2, _n, _v1;

mwoibn::Axis _x_world, _y_world,
             _z_world; // for now assume the basic initialization
double _d_castor;

mwoibn::Vector3 _vA;
mwoibn::Vector3T _tA, _tB;

mwoibn::Matrix3 _mA, _mB, _mC, _s_v1, _s_n, _s_v2;
};

}
} // namespace package
} // namespace library
#endif
