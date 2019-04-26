
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

#include <chrono>

namespace mwoibn {
namespace hierarchical_control {
namespace actions {


class ShapeAction : public Primary {
public:
ShapeAction(mgnss::higher_level::QpAggravated& task, mwoibn::hierarchical_control::tasks::ContactPoint& contact_point,
            std::vector<mwoibn::hierarchical_control::tasks::Angle>& steering, mgnss::higher_level::SteeringReference& steering_reference,
            mwoibn::hierarchical_control::tasks::Aggravated& aggravated, mwoibn::hierarchical_control::tasks::Aggravated& angles,
            mwoibn::hierarchical_control::tasks::Aggravated& caster, mwoibn::hierarchical_control::tasks::Aggravated& camber,
             mgnss::higher_level::StateMachine& state_machine,
            hierarchical_control::State& state, mwoibn::Vector3& next_step, double dt, mwoibn::robot_class::Robot& robot) :
            Primary(task, state.memory), _qr_task(task), _contact_point(contact_point), _steering(steering),
            _steering_reference(steering_reference), _aggravated(aggravated), _angles(angles), _caster(caster), _camber(camber),
             _state_machine(state_machine), _dt(dt), _next_step(next_step), _robot(robot){

               _desired_steer.setZero(4);
               _current_steer.setZero(4);
               _modified_support.setZero(8);
               _support_world.setZero(8);
               _support.setZero(12);
               _eigen_scalar.setZero(1);
               _state.setZero(_robot.getDofs());
               _robot.states.add(QR, _robot.getDofs());
               // _robot.command.add(QR_TASK_VELOCITY, _robot.getDofs());
               // _robot.command.add("QR_TASK_POSITION", _robot.getDofs());
               // _robot.command.add("QR_TASK_TORQUE", _robot.getDofs());
               // _robot.command.add("QR_TASK_ACCELERATION", _robot.getDofs());

               temp_des.setZero(12);
               temp_qr.setZero(12);

}

virtual void init(){
    _angles.update();
    _state_machine.update();
    _qr_task._update();
    _qr_task.solve();
}

virtual void run(){

    _angles.update();
    _state_machine.update();

//    _qr_task._update();

//    now = std::chrono::high_resolution_clock::now();
    _qr_task.solve();
//    end = std::chrono::high_resolution_clock::now();
//    elapsed_solve = end - now;

    _robot.states[QR].velocity.set(_qr_task.raw().head(_qr_task.activeDofs().size()), _qr_task.activeDofs());

    if(std::isinf(_qr_task.optimalCost())){
        ++infs;
        _robot.states[QR].velocity.set(mwoibn::VectorN::Zero(_robot.states[QR].velocity.size()));
    }

    for(int i =0; i < 4; i++){
        _eigen_scalar[0] = 0;
        for(int k = 0; k < _qr_task.activeDofs().size(); k++)
          _eigen_scalar.noalias() +=  _steering[i].getJacobian().col(_qr_task.activeDofs()[k])*_qr_task.raw()[k];
        _desired_steer[i] = -2*_eigen_scalar[0]*_dt;
        _current_steer[i] = _steering[i].getCurrent();
        // _current_steer[i] = _steering[i].getReference();

    }


    // mwoibn::VectorN _modified_support  =  _state_machine.stateJacobian()*_qr_task.raw().head(_contact_point.getJacobian().cols());
    // mwoibn::VectorN _modified_support(8);
    _state.setZero();
    for(int i = 0; i < _qr_task.activeDofs().size(); i++)
      _state[_qr_task.activeDofs()[i]] = _qr_task.raw()[i];
    _qr_task.task(0).set(_state);
    _qr_task.task(0).transform();
    _support_world.noalias()  =  _state_machine.stateJacobian()*_qr_task.task(0).get();
    _support_world +=  _state_machine.stateOffset();

    for(int i =0; i < 4; i++){
      test__ = mwoibn::Vector3::Zero();
      test__.head<2>() = _support_world.segment<2>(2*i);
      _modified_support.segment<2>(2*i) = (_state_machine.steeringFrames()[i]->rotation*test__).head<2>();
    }

    if(std::isinf(_qr_task.optimalCost())){
      _desired_steer.setZero();
      _modified_support.setZero();
      _support_world.setZero();
    }


    //
    for(int i =0; i < 4; i++){
      _support.segment<3>(3*i) = _contact_point.getReferenceWorld(i);
      _support.segment<2>(3*i).noalias() += _modified_support.segment<2>(2*i)*_dt;
    }

    temp_qr = _support;
    //
    mwoibn::Vector3 support_i;
    for(int i =0; i < 4; i++){
      support_i = _support.segment<3>(3*i);
      _contact_point.setReferenceWorld(i, support_i, false);
    }


    _steering_reference.compute(_next_step, _desired_steer, _current_steer);
    //
    for(int i =0; i < 4; i++)
      _steering[i].setReference(_steering_reference.get()[i]);

    temp_des.setZero(12);
    for(int i =0; i < 4; i++){
      support_i = _support.segment<3>(3*i);
      support_i.head<2>() -= _modified_support.segment<2>(2*i)*_dt;

      vel__.setZero();
      vel__.head<2>() = _modified_support.segment<2>(2*i);

      test__.noalias() = _state_machine.steeringFrames()[i]->rotation.transpose()*vel__*_dt;
      test__[1] = 0;
      support_i.noalias() += _state_machine.steeringFrames()[i]->rotation*test__;
      temp_des.segment<3>(3*i) = support_i;

      _contact_point.setReferenceWorld(i, support_i, false);
    }


      for(int i = 0; i < 4; i++){

        _camber.setWeight(1, i);
        _caster.setWeight(1, i);
        // _leg_tasks["CASTER"].first.setWeight(0.5, i);
      }

      _caster.update();


        // I need to change when the tasks are updated
    for(int i = 0; i < 4; i++){
      // _weight[i] = _camber.getWeight(i);
      double weight = std::fabs( _camber.getTask(i).getJacobian()(0, 6*(i+1)+3) / _camber.getTask(i).getJacobian()(0, 6*(i+1)+4)  );
      // double weight_2 = std::fabs( _tasks["CONTACT_POINTS_1"]->getJacobian()(2*i+1, 6*(i+1)+4) / _tasks["CONTACT_POINTS_1"]->getJacobian()(2*i+1, 6*(i+1)+0)  );

      // _camber.setWeight(std::tanh( 0.1*std::pow(weight,3)), i);
      // _caster.setWeight(1-std::tanh( 0.1*std::pow(weight,3) ), i);
      _camber.setWeight(1, i);
      _caster.setWeight(0, i);
      // _caster.setWeight(0.0, i);
      // std::cout << "caster//camber/t" << _caster.getWeight(i) << "\t" << _camber.getWeight(i) << std::endl;

      // _leg_tasks["CASTER"].first.setWeight(0.5, i);
    }


    _contact_point.update();
    _aggravated.update(); // steering

}

virtual void release(){ }

void log(  mwoibn::common::Logger& logger){
//  for(int i =0 ; i < 4; i++){
//    logger.add("cp_des_"   + std::to_string(i) + "_x", temp_des[3*i] );
//    logger.add("cp_des_"   + std::to_string(i) + "_y", temp_des[3*i+1] );
//    logger.add("cp_qr_"   + std::to_string(i) + "_x", temp_qr[3*i] );
//    logger.add("cp_qr_"   + std::to_string(i) + "_y", temp_qr[3*i+1] );
//    logger.add("v_steer_"   + std::to_string(i), _desired_steer[i]/_dt );
//    logger.add("current_"   + std::to_string(i), _current_steer[i]*180/mwoibn::PI );
//  }
    if(std::isinf(_qr_task.optimalCost()))
        logger.add("cost_", -mwoibn::NON_EXISTING );
    else
        logger.add("cost_", _qr_task.optimalCost() );

//    logger.add("elapsed_solve", elapsed_solve.count()     );


}

protected:
  // hierarchical_control::State& _state;
  mgnss::higher_level::QpAggravated& _qr_task;
  mwoibn::hierarchical_control::tasks::ContactPoint &_contact_point;
  std::vector<mwoibn::hierarchical_control::tasks::Angle> &_steering;
  mgnss::higher_level::SteeringReference& _steering_reference;
  mwoibn::hierarchical_control::tasks::Aggravated& _aggravated; // steering
  mwoibn::hierarchical_control::tasks::Aggravated &_angles, &_caster, &_camber;
  // mgnss::higher_level::QrTask& _unconstrainted;
  mgnss::higher_level::StateMachine& _state_machine;
  mwoibn::VectorN _state;
  mwoibn::VectorN _weight;
  mwoibn::VectorN _desired_steer, _modified_support, _support_world, _support, _current_steer;
  mwoibn::Vector3 test__, vel__;
  mwoibn::VectorN _eigen_scalar;
  double _dt;
  mwoibn::Vector3& _next_step;
  mwoibn::robot_class::Robot& _robot;
  int infs = 0;
  mwoibn::VectorN temp_qr, temp_des;
  const std::string QR = "QR";

//  std::chrono::time_point<std::chrono::high_resolution_clock> now, end;
//  std::chrono::duration<double> elapsed_solve;
};

}
} // namespace package
} // namespace library
#endif
