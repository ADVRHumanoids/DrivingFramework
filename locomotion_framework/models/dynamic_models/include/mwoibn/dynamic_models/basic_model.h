#ifndef __MWOIBN__DYANMIC_MODELS__BASIC_MODEL_H
#define __MWOIBN__DYANMIC_MODELS__BASIC_MODEL_H

#include <rbdl/rbdl.h>
#include "mwoibn/robot_class/robot.h"
#include "mwoibn/dynamic_models/dynamic_models.h"
#include "mwoibn/common/update_manager.h"

namespace mwoibn
{

namespace dynamic_models
{

/** @brief This class provides a base for all dynamic models
 *
 * It directly implements RBDL methods for a robot model.
 *
 **/
class BasicModel
{

public:
BasicModel(mwoibn::robot_class::Robot& robot, std::initializer_list<robot_class::DYNAMIC_MODEL> update = {}) : _robot(robot),
           _manager(*this, {{robot_class::DYNAMIC_MODEL::GRAVITY, &dynamic_models::BasicModel::_updateGravity},
                     {robot_class::DYNAMIC_MODEL::INERTIA, &dynamic_models::BasicModel::_updateInertia},
                     {robot_class::DYNAMIC_MODEL::NON_LINEAR, &dynamic_models::BasicModel::_updateNonlinearEffects}})
            {

        _zero.setZero(_robot.getDofs());
        _gravity.setZero( _robot.getDofs());
        _non_linear.setZero(_robot.getDofs());
        _inertia.setZero(_robot.getDofs(),
                         _robot.getDofs());

        _manager.subscribe(update);

}

virtual ~BasicModel() {
}



mwoibn::robot_class::Robot& getRobot() const {
        return _robot;
}

virtual const mwoibn::VectorN& getGravity()
{
        return _gravity;
}

/** @brief returns all modeled nonlinear effects including gravity in robots
 * dynamic **/
virtual const mwoibn::VectorN& getNonlinearEffects()
{
        return _non_linear;
}

/** @brief returns inertia matrix**/
virtual const mwoibn::Matrix& getInertia()
{
        return _inertia;
}

virtual void update(){_manager.update();}
virtual void unsubscribe(robot_class::DYNAMIC_MODEL interface){_manager.unsubscribe(interface);}
virtual void subscribe(robot_class::DYNAMIC_MODEL interface){_manager.subscribe(interface);}
virtual void subscribe(std::vector<robot_class::DYNAMIC_MODEL> interface){_manager.subscribe(interface);}
virtual void isSubscribed(robot_class::DYNAMIC_MODEL interface){_manager.is(interface);}


//const mwoibn::common::UpdateManager<robot_class::DYNAMIC_MODEL, dynamic_models::QrDecomposition>& manager() const {return _manager;}


protected:
mwoibn::robot_class::Robot& _robot;
mwoibn::VectorN _gravity, _non_linear, _zero;
mwoibn::Matrix _inertia;
mwoibn::common::UpdateManager<robot_class::DYNAMIC_MODEL, dynamic_models::BasicModel> _manager;

/** @brief returns gravity effect computed **/
virtual void _updateGravity()
{
        _gravity.setZero();
        RigidBodyDynamics::NonlinearEffects(_robot.getModel(),
                                            _robot.state.position.get(), _zero, _gravity);

}

/** @brief returns all modeled nonlinear effects including gravity in robots
 * dynamic **/
virtual void _updateNonlinearEffects()
{

        _non_linear.setZero();
        RigidBodyDynamics::NonlinearEffects(_robot.getModel(),
                                            _robot.state.position.get(),
                                            _robot.state.velocity.get(), _non_linear);
}

/** @brief returns inertia matrix**/
virtual void _updateInertia()
{
        _inertia.setZero();

        RigidBodyDynamics::CompositeRigidBodyAlgorithm(
                _robot.getModel(), _robot.state.position.get(), _inertia, false);

}



};
} // namespace package
} // namespace library

#endif // DYANMIC_MODELS_BASIC_MODEL_H
