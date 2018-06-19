#ifndef __MWOIBN_ROBOT_CLASS_CAMBER_H
#define __MWOIBN_ROBOT_CLASS_CAMBER_H

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

class Camber : public Basic
{
public:
Camber(mwoibn::robot_class::Robot& robot,
       mwoibn::point_handling::Point point,
       mwoibn::Axis axis)
        : Basic(robot, point, axis)
{
        _x_world << 1,0,0;
        _y_world << 0,1,0;
        _z_world << 0,0,1;
        _J.setZero(1, robot.getDofs());

}   // for now just support major axes

virtual ~Camber() {
}

virtual void update()
{
        _y = _point.getRotationWorld(_robot.state.get()) *
             _axis;

        _n = _y.cross(_z_world);

        mwoibn::Scalar b = 1/_n.norm();

        _n.normalize();

        _v1 = _z_world - _y * _z_world.transpose() * _y;
        _v1.normalize();

        _v2 = _z_world;

        mwoibn::Scalar cross = (_v1.cross(_v2)).transpose() * _n;
        mwoibn::Scalar dot = _v1.transpose() * _v2;

        _angle = std::atan2(cross, dot);

// DERIVATIVE

        mwoibn::Scalar H = cross*cross + dot * dot;

        mwoibn::Scalar I = dot / H;
        H = -cross / H;

        mwoibn::eigen_utils::skew(_y, _s_y);
        mwoibn::eigen_utils::skew(_v2, _s_v2);

        _mB  = _y*_z_world.transpose();
        _vA  = _mB*_y;

        mwoibn::eigen_utils::skew(_vA, _mA);

        _mA += _mB*_s_y;

        _mB  = 0.5*b*b*_z_world*_z_world.transpose();
        _mB -= mwoibn::Matrix3::Identity();
        _mB -= 0.5*b*b*_y*_y.transpose()*_z_world*_z_world.transpose();
        _mB  = -_mB*b;

        _mC  = _mB *_mA;

        _tA  = -0.5*b*b*b * _z_world.transpose()*_mA;

        _mB  = _s_y*_v2*_tA;
        _mB += b*_s_v2*_s_y;

        _tA  = _v2.transpose()*_mC;
        _tB  = -_n.transpose()*_s_v2*_mC;
        _tB -= (_s_v2*_v1).transpose()*_mB;

        _tA  = H*_tA;
        _tA += I*_tB;

        _J.noalias() = _tA * _point.getOrientationJacobian(_robot.state.get());

}

protected:
mwoibn::Axis _v2, _n, _v1, _y;

mwoibn::Axis _x_world, _y_world,
             _z_world; // for now assume the basic initialization
double _d_camber;

mwoibn::Vector3 _vA;
mwoibn::Vector3T _tA, _tB;

mwoibn::Matrix3 _mA, _mB, _mC,  _s_y, _s_n, _s_v2;

};

}
} // namespace package
} // namespace library

#endif
