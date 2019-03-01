#ifndef __MWOIBN_HIERARCHICAL_CONTROL_TASKS_CONTACT_POINT_SECOND_ORDER_H
#define __MWOIBN_HIERARCHICAL_CONTROL_TASKS_CONTACT_POINT_SECOND_ORDER_H

#include "mwoibn/hierarchical_control/hierarchical_control.h"
#include "mwoibn/hierarchical_control/tasks/contact_point.h"
#include "mwoibn/robot_points/handler.h"
#include "mwoibn/robot_points/point.h"
#include "mwoibn/dynamic_points/dynamic_point.h"
#include "mwoibn/robot_points/rotation.h"
#include "mwoibn/robot_points/minus.h"
#include "mwoibn/common/update_manager.h"

#include "mwoibn/hierarchical_control/tasks/center_of_mass_task.h"

#include "mwoibn/robot_points/ground_wheel.h"
#include "mwoibn/robot_points/torus_model.h"
#include "mwoibn/dynamic_points/torus_velocity.h"


namespace mwoibn
{
namespace hierarchical_control
{
namespace tasks
{

/**
 * @brief The CartesianWorld class Provides the inverse kinematics task
 ********to control the position of a point defined in one of a robot reference frames
 *
 */
class ContactPointSecondOrder : public ContactPoint
{

public:
/**
 * @param[in] ik the point handler mamber that defines which point is
 ********controlled by this task instance it makes a local copy of a point handler to
 ********prevent outside user from modifying a controlled point
 *
 */
ContactPointSecondOrder(std::vector<std::string> names, mwoibn::robot_class::Robot& robot, YAML::Node config,
                        mwoibn::robot_points::Point& base_point, std::string base_link)
        : ContactPoint(base_point), _robot(robot), _contacts(_robot.getDofs()), _base_point(base_point),
         _base( base_link, robot.getModel(), robot.state), _base_ang_vel(_base),
         _minus(_robot.getDofs()), _ground_normal(_robot.contacts()[0].getGroundNormal())
{
      _update.push_back(_manager.signIn(std::bind(&ContactPointSecondOrder::_updateError, this)));
      _update.push_back(_manager.signIn(std::bind(&ContactPointSecondOrder::_updateJacobian, this)));
      _update.push_back(_manager.signIn(std::bind(&ContactPointSecondOrder::_updateState, this)));


      for(auto& contact: _robot.contacts())
      {
          std::string name = _robot.getBodyName(contact->wrench().getBodyId());
          if(!std::count(names.begin(), names.end(), name)){
            std::cout << "Tracked point " << name << " could not be initialized" << std::endl;
            names.erase(std::remove(names.begin(), names.end(), name), names.end());
            continue;
          }

          std::unique_ptr<mwoibn::robot_points::TorusModel> torus_(new mwoibn::robot_points::TorusModel(
                             _robot.getModel(), _robot.state, mwoibn::point_handling::FramePlus(name,
                             _robot.getModel(), _robot.state),
                             mwoibn::Axis(config["reference_axis"][name]["x"].as<double>(),
                                          config["reference_axis"][name]["y"].as<double>(),
                                          config["reference_axis"][name]["z"].as<double>()),
                                          config["minor_axis"].as<double>(), config["major_axis"].as<double>(),
                                          contact->getGroundNormal()));

          _wheel_transforms.push_back(std::unique_ptr<mwoibn::robot_points::Rotation>(
                    new mwoibn::robot_points::GroundWheel(torus_->axis(), torus_->groundNormal())));
          _support.add(std::move(torus_));
          _contacts.add(mwoibn::dynamic_points::TorusVelocity(_support.end(-1), _robot));

      }

          _allocate();
          reset();
}

void subscribe(bool error, bool jacobian, bool state){
    if(error) {_update[0]->subscribe(); _update[2]->subscribe();}
    if(jacobian) {_update[1]->subscribe(); _update[2]->subscribe();}
    if(state) _update[2]->subscribe();
}

void unsubscribe(bool error, bool jacobian, bool state){
    if(error) {_update[0]->unsubscribe(); _update[2]->unsubscribe();}
    if(jacobian) {_update[1]->unsubscribe(); _update[2]->unsubscribe();}
    if(state) _update[2]->unsubscribe();
}


virtual void start(){
  _manager.reset();
}

virtual void reset()
{

  _updateState();

  for (int i = 0; i < _contacts.size(); i++)
  {
    _reference.segment<3>(3 * i) = getPointStateReference(i);
  }
}


virtual void updateState() final {
      _updater(_update[2]);
  }

  virtual void updateError() final {
        updateState();
        _updater(_update[0]);
    }
  virtual void updateJacobian() final {
          updateState();
          _updater(_update[1]);
  }


  virtual const mwoibn::VectorN& getReference() const {
          return _reference;
  }

  virtual void setReference(const mwoibn::VectorN& reference)
  {
          for (int i = 0; i < _reference.size(); i++)
                  _reference[i] = reference[i];
  }

  virtual mwoibn::VectorN getReference(int i) const
  {
    return _reference.segment<3>(i * 3);
  }

  virtual void setReference(int i, const mwoibn::Vector3& reference)
  {
    _reference.segment(i * 3, 3) = reference;
  }


    virtual void setReferenceWorld(int i, const mwoibn::Vector3& reference,
                                   bool update)
    {

      if (update)
        updateState();

      _reference.segment<3>(i*3) = _worldToBase(reference);

    }

    virtual mwoibn::Vector3
    getReferenceWorld(int i) // it can have update as it uses RBDL
                                          // call and cannot be constant anyway
    {

      mwoibn::Vector3 reference;
      reference = _reference.segment(i * 3, 3);

      return _baseToWorld(reference);
    }

    virtual const mwoibn::Vector3& getVelocityReference(int i)
    {
      _point.noalias() = _worldToBase(_wheel_transforms[i]->rotation*_velocity.segment<3>(3*i));
      return _point;
    }
    virtual const mwoibn::Vector3& getPointStateReference(int i)
    {
      _point.noalias() = _worldToBase(_contacts[i].get());
      return _point;
    }

    virtual const mwoibn::Vector3& getReferenceError(int i)
    {
      _point.noalias() = _q_twist.transposed().rotate(_full_error.segment<3>(3 * i));
      return _point;
    }

const mwoibn::VectorN& getFullError(){return _full_error;}

const mwoibn::VectorN& getWorldError() const {
        return _error;
}

virtual const mwoibn::VectorN& getForce(){return _force;}

virtual void releaseContact(int i) { _selector[i] = true; }
virtual void claimContact(int i) { _selector[i] = false; }

void setVelocity(mwoibn::VectorN& velocity){
  for (int i = 0; i < _contacts.size(); i++)
    _velocity_ref.segment<3>(3*i) = _wheel_transforms[i]->rotation.transpose()*velocity.segment<3>(3*i);
}

virtual double heading(){
      return _q_twist.angle();
}

virtual double baseX(){
      return base.get()[0];
}

virtual double baseY(){
      return base.get()[1];
}

protected:
  mwoibn::robot_class::Robot& _robot;

  mwoibn::robot_points::Handler<mwoibn::dynamic_points::DynamicPoint> _contacts;
  mwoibn::robot_points::Handler<mwoibn::robot_points::TorusModel> _support;
  mwoibn::robot_points::Point& _base_point;
  mwoibn::point_handling::FramePlus _base;
  mwoibn::point_handling::AngularVelocity _base_ang_vel;
  std::vector<std::unique_ptr<mwoibn::robot_points::Rotation>> _wheel_transforms;
  mwoibn::robot_points::Handler<mwoibn::robot_points::Minus> _minus;

  mwoibn::VectorBool _selector;

  mwoibn::Vector3 _point;
  mwoibn::VectorN _reference, _full_error, _force, _velocity_ref;
  mwoibn::Quaternion _q_twist;
  const mwoibn::Vector3& _ground_normal;
  mwoibn::Matrix3 _rot, _projected, _rot_project;

  mwoibn::update::UpdateManager _manager;
  std::vector<std::shared_ptr<mwoibn::update::Function>> _update;
  void _updater(std::shared_ptr<mwoibn::update::Function>& func){
    if(func->done()) func->call();
    func->count();

  }

  virtual void _updateError()
  {
    // std::cout << "_updateError" << std::endl;
    _last_error.noalias() = _error; // save previous state

    for (int i = 0; i < _contacts.size(); i++)
    {

      _full_error.segment<3>(3*i) = _q_twist.rotate(_reference.segment<3>(i*3)) - _minus[i].get();
      _error.segment<3>(3 * i).noalias() = _wheel_transforms[i]->rotation.transpose()*_full_error.segment<3>(3*i); // 10 is for a task gain should be automatic

      if (_selector[i])
        _error[3*i+2] = 0;

      _force.segment<3>(3*i).noalias() =  _wheel_transforms[i]->rotation.transpose()*(_robot.contacts()[i].wrench().force.getWorld());

      _velocity.segment<3>(3*i) = _velocity_ref.segment<3>(3*i) + _contacts[i].getConstant();
    }

    std::cout << "_error\t" << _error.transpose() << std::endl;
    std::cout << "_velocity\t" << _velocity.transpose() << std::endl;

  }


  virtual void _updateJacobian()
  {

    _last_jacobian.noalias() = _jacobian;

    for (int i = 0; i < _contacts.size(); i++)
    {
      _jacobian.block(3*i, 0, 3, _jacobian.cols()).noalias() = -_wheel_transforms[i]->rotation.transpose()*(_minus[i].getJacobian());
      _projected = _ground_normal*_ground_normal.transpose();
      mwoibn::eigen_utils::skew(_q_twist.rotate(_reference.segment<3>(i*3)), _rot);
      _rot_project = _rot*_projected;
      _rot = _wheel_transforms[i]->rotation.transpose()*_rot_project;
      _jacobian.block(3*i, 0, 3, _jacobian.cols()).noalias() -= _rot*_base_ang_vel.getJacobian();
      if (_selector[i])
        _jacobian.row(3*i+2).setZero();
    }

    std::cout << "_jacobian\n" << _jacobian << std::endl;

  }



  virtual mwoibn::Vector3 _worldToBase(mwoibn::Vector3 point)
  {

          mwoibn::Vector3 basePoint;
          basePoint.noalias() = point;
          basePoint.head<2>() -= _base_point.get().head<2>();

          return _q_twist.transposed().rotate(basePoint);
  }

  virtual mwoibn::Vector3 _baseToWorld(mwoibn::Vector3 point)
  {

          mwoibn::Vector3 basePoint;
          basePoint.noalias() = _q_twist.rotate(point);
          basePoint.head<2>() += _base_point.get().head<2>();

          return basePoint;
  }

  virtual void _allocate(){
    _init(_contacts.rows(), _contacts.cols());
    _selector = mwoibn::VectorBool::Constant( _robot.contacts().size(), true); // on init assume all constacts should be considered in a task
    _reference.setZero(_contacts.rows());
    _full_error.setZero(_contacts.rows());
    _force.setZero(_robot.contacts().size()*3);
    _velocity_ref.setZero(_contacts.rows());

    for(auto& contact: _contacts)
      _minus.add(mwoibn::robot_points::Minus(*contact, _base_point));
  }

    virtual mwoibn::Vector3 _referencePoint(int i)
    {

      mwoibn::Vector3 _temp_point = _reference.segment<3>(3 * i);

      return _baseToWorld(_temp_point);
    }


    virtual void _updateState(){
      _support.update(true);
      _contacts.update(true);
      _q_twist = _base.orientation.getWorld().twistSwing(_ground_normal); // this has heading
      for(auto& wheel: _wheel_transforms)
        wheel->compute();

      _minus.update(true);
    }




};
}
} // namespace package
} // namespace library
#endif
