#ifndef __MWOIBN_HIERARCHICAL_CONTROL_SHAPE_ACTIONS_H
#define __MWOIBN_HIERARCHICAL_CONTROL_SHAPE_ACTIONS_H

#include "mwoibn/hierarchical_control/actions/primary.h"
#include "mwoibn/hierarchical_control/tasks/controller_task.h"
#include "mwoibn/hierarchical_control/state.h"
#include "mgnss/higher_level/qp/tasks/qr_task.h"
#include "mgnss/higher_level/steering_shape.h"
#include "mwoibn/hierarchical_control/tasks/contact_point.h"
#include "mwoibn/hierarchical_control/tasks/aggravated.h"
#include "mwoibn/hierarchical_control/tasks/angle.h"
#include <mgnss/higher_level/state_machine.h>

namespace mwoibn {
namespace hierarchical_control {
namespace actions {


class ShapeAction : public Primary {
public:
ShapeAction(mgnss::higher_level::QrTask& task, mwoibn::hierarchical_control::tasks::ContactPoint& contact_point,
            std::vector<mwoibn::hierarchical_control::tasks::Angle>& steering, mgnss::higher_level::SteeringShape& steering_reference,
            mwoibn::hierarchical_control::tasks::Aggravated& aggravated, mgnss::higher_level::StateMachine& state_machine,
            hierarchical_control::State& state, mwoibn::Vector3& next_step, double dt) :
            Primary(task, state.memory), _state(state), _qr_task(task), _contact_point(contact_point), _steering(steering),
            _steering_reference(steering_reference), _aggravated(aggravated), _state_machine(state_machine), _dt(dt), _next_step(next_step){
    }


virtual void run(){



    _state_machine.update();
    _qr_task._update();
    _qr_task.solve();

    mwoibn::VectorN desired_steer(4);
    // std::cout << "previous\t" << _state.command.transpose() << std::endl;

    // std::cout << "_qr_task\t" << _qr_task.raw().transpose() << std::endl;
    std::cout << "_cost\t" << _qr_task.optimalCost() << std::endl;
    // std::cout << "_steering\t" << _steering[0].getJacobian().transpose() << std::endl;

    for(int i =0; i < 4; i++)
        desired_steer[i] = -(_steering[i].getJacobian()*_qr_task.raw().head(_steering[i].getJacobian().cols()))[0]*_dt;


    // mwoibn::VectorN _modified_support  =  _state_machine.stateJacobian()*_qr_task.raw().head(_contact_point.getJacobian().cols());
    mwoibn::VectorN _modified_support(8);
    mwoibn::VectorN _support_world  =  (_state_machine.stateJacobian()*_qr_task.get() + _state_machine.stateOffset());

    for(int i =0; i < 4; i++){
      mwoibn::Vector3 test__ = mwoibn::Vector3::Zero();
      test__.head<2>() = _support_world.segment<2>(2*i);
      _modified_support.segment<2>(2*i) = (_state_machine.steeringFrames()[i]->rotation*test__).head<2>();
    }
    std::cout << "_modified_support\t" << _modified_support.transpose() << std::endl;
    std::cout << "desired_steer\t" << desired_steer.transpose() << std::endl;

    if(std::isinf(_qr_task.optimalCost())){
      desired_steer.setZero();
      _modified_support.setZero();
    }

    mwoibn::VectorN _support(12);
    //
    for(int i =0; i < 4; i++){
      _support.segment<3>(3*i) = _contact_point.getReferenceWorld(i);
      _support.segment<2>(3*i) += _modified_support.segment<2>(2*i)*_dt;
    //   // std::cout << i << "_modified_support\t" << (_modified_support.segment<2>(2*i)).transpose() << std::endl;
    //   // std::cout << i << "_modified_support\t" << (_modified_support.segment<2>(2*i)*_dt).transpose() << std::endl;
    //   // std::cout << i << "_support\t" << (_support.segment<3>(3*i)*_dt).transpose() << std::endl;
    }
    //
    mwoibn::Vector3 support_i;
    for(int i =0; i < 4; i++){
      support_i = _support.segment<3>(3*i);
      _contact_point.setReferenceWorld(i, support_i, false);
    }

    _steering_reference.compute(_next_step, desired_steer);
    //
    for(int i =0; i < 4; i++)
      _steering[i].setReference(_steering_reference.get()[i]);


    for(int i =0; i < 4; i++){
      support_i = _support.segment<3>(3*i);
      support_i.head<2>() -= _modified_support.segment<2>(2*i)*_dt;
      std::cout << i << "\t_modified_support\t" << (_modified_support.segment<2>(2*i)*_dt).transpose() << std::endl;
      std::cout << i << "\tsupport_i\t" << support_i.transpose() << std::endl;
      mwoibn::Vector3 vel__;
      vel__.setZero();
      vel__.head<2>() = _modified_support.segment<2>(2*i);
      mwoibn::Vector3 test__ = _state_machine.steeringFrames()[i]->rotation.transpose()*vel__*_dt;
      // std::cout << "test\t" << test__.transpose() << std::endl;
      test__[1] = 0;
      support_i += _state_machine.steeringFrames()[i]->rotation*test__;
      // std::cout << "test\t" << (_state_machine.steeringFrames()[i]->rotation*test__).transpose() << std::endl;
      std::cout << i << "\tsupport_i\t" << support_i.transpose() << std::endl;
      _contact_point.setReferenceWorld(i, support_i, false);
    }

      std::cout << "_steering_reference.get()\t" << _steering_reference.get().transpose() << std::endl;

    _contact_point.update();
    _aggravated.update();

}

virtual void release(){ }

protected:
  hierarchical_control::State& _state;
  mgnss::higher_level::QrTask& _qr_task;
  mwoibn::hierarchical_control::tasks::ContactPoint &_contact_point;
  std::vector<mwoibn::hierarchical_control::tasks::Angle> &_steering;
  mgnss::higher_level::SteeringShape& _steering_reference;
  mwoibn::hierarchical_control::tasks::Aggravated& _aggravated;
  mgnss::higher_level::StateMachine& _state_machine;

  double _dt;
  mwoibn::Vector3& _next_step;
};

}
} // namespace package
} // namespace library
#endif
