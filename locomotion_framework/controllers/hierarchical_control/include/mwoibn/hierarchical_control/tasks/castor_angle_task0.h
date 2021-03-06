#ifndef __MWOIBN_HIERARCHICAL_CONTROL_TASKS_CASTOR_ANGLE_TASK_H
#define __MWOIBN_HIERARCHICAL_CONTROL_TASKS_CASTOR_ANGLE_TASK_H

#include "mwoibn/hierarchical_control/tasks/controller_task.h"
#include "mwoibn/hierarchical_control/hierarchical_control.h"
#include "mwoibn/point_handling/robot_points_handler.h"
//#include <rbdl/rbdl.h>

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

class CastorAngle
{
public:
CastorAngle(mwoibn::robot_class::Robot& robot,
            mwoibn::point_handling::Position point, mwoibn::Axis axis)
        : _point(point), _robot(robot), _axis(axis)
{
        _x_world << 1, 0, 0;
        _y_world << 0, 1, 0;
        _z_world << 0, 0, 1;
        _J.setZero(1, robot.getDofs());

}   // for now just support major axes

virtual ~CastorAngle() {
}

void update()
{
        // axis - z
        _v1 = _point.getRotationWorld() * _z_world;

        _n = _point.getRotationWorld() * _axis;

        _v2 = _z_world - _n * _z_world.transpose() * _n;

        double b = 1 / _v2.norm();
        _v2.normalize();

        double cross = (_v1.cross(_v2)).transpose() * _n;
        double dot = _v1.transpose() * _v2;
        _castor = std::atan2(cross, dot);

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

        _J.noalias() = _tA * _point.getOrientationJacobian();
}

double get() {
        return _castor;
}
const mwoibn::Matrix& getJacobian() {
        return _J;
}

protected:
mwoibn::Axis _axis;
mwoibn::Axis _v2, _n, _v1;

mwoibn::Axis _x_world, _y_world,
             _z_world; // for now assume the basic initialization
double _castor, _d_castor;
mwoibn::point_handling::Position _point;
mwoibn::robot_class::Robot& _robot;

mwoibn::Vector3 _vA;
mwoibn::Vector3T _tA, _tB;

mwoibn::Matrix3 _mA, _mB, _mC, _s_v1, _s_n, _s_v2;

mwoibn::Matrix _J;
mwoibn::Matrix3 _skew(mwoibn::Vector3 vec)
{

        mwoibn::Matrix3 mat;
        mat << 0, -vec[2], vec[1], vec[2], 0, -vec[0], -vec[1], vec[0], 0;

        return mat;
}
};

class CastorAngleTask : public BasicTask
{

public:
CastorAngleTask(std::vector<hierarchical_control::tasks::CastorAngle> angels,
                mwoibn::robot_class::Robot& robot)
        : BasicTask(), _robot(robot), _angles(angels)
{
        _init(_angles.size(), _robot.getDofs());
        _ref.setZero(_angles.size());
        _current.setZero(_angles.size());
}

virtual ~CastorAngleTask() {
}

virtual void updateError()
{
        //    std::cout << "update" << std::endl;
        _last_error.noalias() = _error;

        for (int i = 0; i < _angles.size(); i++)
        {
                _angles[i].update();
                _error[i] = _ref[i] - _angles[i].get();
                //      std::cout << "\t" << _angels[i].get()*180/3.14;// << std::endl;
                //      std::cout << "\t" << _error[i]*180/3.14;// << std::endl;
        }

        //    std::cout << "\t" << _angels[1].get()*180/3.14;// << std::endl;
        //    std::cout << "\t" << _error[1]*180/3.14;
        eigen_utils::limitToHalfPi(
                _error); // make a bigger limit to avoid chattering
                         //    std::cout << "\t" << _error[1]*180/3.14;// << std::endl;

        //    std::cout << std::fixed << "error\t" << _error.transpose() * 180 /
        //    3.14 << "\n";
        //    std::cout << std::endl;
}

virtual void updateJacobian()
{
        _last_jacobian.noalias() = _jacobian;

        for (int i = 0; i < _angles.size(); i++)
        {
                _jacobian.row(i) = -_angles[i].getJacobian();
        }
}

const mwoibn::VectorN& getCurrent()
{

        for (int i = 0; i < _angles.size(); i++)
                _current[i] = _angles[i].get();

        return _current;
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
int size() {
        return _angles.size();
}

protected:
mwoibn::robot_class::Robot& _robot;
mwoibn::VectorN _ref, _current;
std::vector<hierarchical_control::tasks::CastorAngle> _angles;
};
}
} // namespace package
} // namespace library
#endif
