#ifndef __MWOIBN_HIERARCHICAL_CONTROL_TASKS_CATRESIAN_SIMPLIFIED_PELVIS_6_H
#define __MWOIBN_HIERARCHICAL_CONTROL_TASKS_CATRESIAN_SIMPLIFIED_PELVIS_6_H

#include "mwoibn/hierarchical_control/tasks/contact_point_3D_rbdl_task.h"

namespace mwoibn
{
namespace hierarchical_control
{
namespace tasks
{
/**
 * @brief The CartesianWorld class Provides the inverse kinematics task
 ***to control the position of a point defined in one of a robot reference frames
 *
 */
class CartesianFlatReference3 : public ContactPoint3DRbdl
{

public:
/**
 * @param[in] ik the point handler mamber that defines which point is
 ***controlled by this task instance it makes a local copy of a point handler to
 ***prevent outside user from modifying a controlled point
 *
 */
CartesianFlatReference3(point_handling::PositionsHandler ik,
                        mwoibn::robot_class::Robot& robot,
                        mwoibn::robot_class::Robot& full_robot)
        : ContactPoint3DRbdl(ik, robot), _full_robot(full_robot)
{

        _map = _robot.biMaps().get("full_body").get();
        init();
}

virtual ~CartesianFlatReference3() {
}


protected:
mwoibn::robot_class::Robot& _full_robot;
mwoibn::VectorInt _map;

virtual mwoibn::Vector3 _contactPoint(int i)
{
        return _ik.getPointStateWorld(i);
}


virtual const mwoibn::Matrix& _referenceJacobian(int i){
        _temp_jacobian.setZero();

        for (int k = 0; k < _map.size(); k++)
        {
                if (_map[k] != mwoibn::NON_EXISTING)
                        _temp_jacobian.col(k) = _full_robot.centerOfMass()
                                                .getJacobian()
                                                .col(_map[k]);
        }

        _temp_jacobian.bottomRows<1>().setZero(); // ?

        return _temp_jacobian;


}

virtual const mwoibn::Matrix& _contactJacobian(int i) {

        _temp_jacobian.noalias() = _ik.getFullPointJacobian(i);
        return _temp_jacobian;

}

virtual mwoibn::Vector3 _worldToBase(mwoibn::Vector3 point){

        mwoibn::Vector3 basePoint;
        basePoint.noalias() = point;
        basePoint.head<2>() -= _full_robot.centerOfMass().get().head<2>();

        return _getTransform().transpose()*basePoint;
}


virtual mwoibn::Vector3 _baseToWorld(mwoibn::Vector3 point){

        mwoibn::Vector3 basePoint;
        basePoint.noalias() = _getTransform() * point;
        basePoint.head<2>() += _full_robot.centerOfMass().get().head<2>();

        return basePoint;

}

virtual void _updateTransform(){
        _transform << std::cos(_state[2]), -std::sin(_state[2]), 0,
        std::sin(_state[2]), std::cos(_state[2]), 0, 0, 0, 1; // shouldn't this be transposed?
}

};
}
} // namespace package
} // namespace library
#endif
