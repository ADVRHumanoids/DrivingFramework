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
 enum class DYNAMIC_MODEL
 {
   INERTIA,
   GRAVITY,
   NON_LINEAR,
   INERTIA_INVERSE
 };


class BasicModel
{

public:
BasicModel(mwoibn::robot_class::Robot& robot, std::initializer_list<DYNAMIC_MODEL> update = {}) : _robot(robot)
            {

        _zero.setZero(_robot.getDofs());
        _gravity.setZero( _robot.getDofs());
        _non_linear.setZero(_robot.getDofs());
        _inertia.setZero(_robot.getDofs(),
                         _robot.getDofs());
        _inverser_ptr.reset(new mwoibn::Inverse(_inertia, 0.000));

        for(auto& entry_: _update_map)
          _function_map[entry_.first] = (_manager.signIn(entry_.second.first)); // register functions

        subscribe(update);


}

virtual ~BasicModel() {
}



mwoibn::robot_class::Robot& getRobot() const {
        return _robot;
}

virtual const mwoibn::VectorN& getGravity()
{
        _function_map[DYNAMIC_MODEL::GRAVITY]->count();
        return _gravity;
}

/** @brief returns all modeled nonlinear effects including gravity in robots
 * dynamic **/
virtual const mwoibn::VectorN& getNonlinearEffects()
{
        _function_map[DYNAMIC_MODEL::NON_LINEAR]->count();
        return _non_linear;
}

/** @brief returns inertia matrix**/
virtual const mwoibn::Matrix& getInertia()
{
        _function_map[DYNAMIC_MODEL::INERTIA]->count();
        return _inertia;
}

virtual const mwoibn::Matrix& getInertiaInverse()
{
        _function_map[DYNAMIC_MODEL::INERTIA_INVERSE]->count();
        return _inverser_ptr->get();
}

virtual void update(){_manager.update();} // Do I want to log status somewhere?
virtual void unsubscribe(DYNAMIC_MODEL interface){
  unsubscribe(_update_map[interface].second);
  _function_map[interface]->unsubscribe();}

virtual void subscribe(DYNAMIC_MODEL interface){
  subscribe(_update_map[interface].second);
  _function_map[interface]->subscribe();}

virtual void unsubscribe(std::vector<DYNAMIC_MODEL> interfaces){
    for(auto interface: interfaces)
      unsubscribe(interface);
    }


virtual void subscribe(std::vector<DYNAMIC_MODEL> interfaces){
  for(auto interface: interfaces)
    subscribe(interface);
  }

protected:
mwoibn::robot_class::Robot& _robot;
mwoibn::VectorN _gravity, _non_linear, _zero;
mwoibn::Matrix _inertia;
mwoibn::update::UpdateManager _manager;
std::unique_ptr<mwoibn::Inverse> _inverser_ptr;

std::map<DYNAMIC_MODEL, std::pair< std::function<void()>, std::vector<DYNAMIC_MODEL> > > _update_map = {
          {DYNAMIC_MODEL::GRAVITY, {std::bind(&dynamic_models::BasicModel::_updateGravity, this),{} } },
          {DYNAMIC_MODEL::INERTIA, {std::bind(&dynamic_models::BasicModel::_updateInertia, this),{} } },
          {DYNAMIC_MODEL::NON_LINEAR, {std::bind(&dynamic_models::BasicModel::_updateNonlinearEffects, this), {} } },
          {DYNAMIC_MODEL::INERTIA_INVERSE, {std::bind(&dynamic_models::BasicModel::_updateInertiaInverse, this), {DYNAMIC_MODEL::INERTIA} } }
         };

std::map<DYNAMIC_MODEL, std::shared_ptr<mwoibn::update::Function> > _function_map;

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

virtual void _updateInertiaInverse()
{
      _function_map[DYNAMIC_MODEL::INERTIA]->count();

      _inverser_ptr->compute(_inertia);

}

};
} // namespace package
} // namespace library

#endif // DYANMIC_MODELS_BASIC_MODEL_H
