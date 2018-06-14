#ifndef __MWOIBN_HIERARCHICAL_CONTROL_TASKS_CAMBER_ANGLE_TASK_2_H
#define __MWOIBN_HIERARCHICAL_CONTROL_TASKS_CAMBER_ANGLE_TASK_2_H

#include "mwoibn/hierarchical_control/hierarchical_control.h"
#include "mwoibn/hierarchical_control/tasks/controller_task.h"
#include "mwoibn/point_handling/robot_points_handler.h"
#include <rbdl/rbdl.h>

namespace mwoibn
{
namespace hierarchical_control
{
namespace tasks
{
/**
 * @brief The CartesianWorld class Provides the inverse kinematics task
 *****to control the position of a point defined in one of a robot reference frames
 *
 */

class CamberAngle
{
public:
CamberAngle(mwoibn::robot_class::Robot& robot,
            mwoibn::point_handling::Point point,
            mwoibn::Axis axis)
        : _axis(axis), _point(point), _robot(robot)
{
        _x_world << 1,0,0;
        _y_world << 0,1,0;
        _z_world << 0,0,1;
        _J.setZero(1, robot.getDofs());

}   // for now just support major axes

~CamberAngle() {
}

void update()
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

        _camber = std::atan2(cross, dot);

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

mwoibn::Scalar get(){
        return _camber;
}
const mwoibn::Matrix& getJacobian(){
        return _J;
}

protected:
mwoibn::Axis _axis;
mwoibn::Axis _v2, _n, _v1, _y;

mwoibn::Axis _x_world, _y_world,
             _z_world; // for now assume the basic initialization
double _camber, _d_camber;
mwoibn::point_handling::Point _point;
mwoibn::robot_class::Robot& _robot;

mwoibn::Vector3 _vA;
mwoibn::Vector3T _tA, _tB;

mwoibn::Matrix3 _mA, _mB, _mC,  _s_y, _s_n, _s_v2;

mwoibn::Matrix _J;


};

class CamberAngleTask : public BasicTask
{

public:
CamberAngleTask(std::vector<hierarchical_control::tasks::CamberAngle> angels, mwoibn::robot_class::Robot& robot)
        : BasicTask(), _robot(robot), _angles(angels)
{
        _init(_angles.size(), _robot.getDofs());
        _ref.setZero(_angles.size());
        _current.setZero(_angles.size());

}

virtual ~CamberAngleTask() {
}

virtual void updateError()
{
        _last_error.noalias() = _error;

        for(int i = 0; i < _angles.size(); i++) {
                _angles[i].update();
                _error[i] = _ref[i] - _angles[i].get();
        }

        eigen_utils::limitToHalfPi(_error); // make a bigger limit to avoid chattering

}

const mwoibn::VectorN& getCurrent(){

        for(int i = 0; i < _angles.size(); i++)
                _current[i] = _angles[i].get();

        return _current;
}

virtual void updateJacobian() {
        _last_jacobian.noalias() = _jacobian;

        for(int i = 0; i < _angles.size(); i++) {
                _jacobian.row(i) = -_angles[i].getJacobian();
        }
}

virtual const mwoibn::VectorN& getReference() const {
        return _ref;
}

virtual void setReference(const mwoibn::VectorN& reference)
{
        _ref = reference;
}

virtual double getReference(int i) const {
        return _ref[i];
}
virtual void setReference(int i, double reference) {
        _ref[i] = reference;
}

protected:
mwoibn::robot_class::Robot& _robot;
mwoibn::VectorN _ref, _current;
std::vector<hierarchical_control::tasks::CamberAngle> _angles;
};

}
} // namespace package
} // namespace library

#endif
