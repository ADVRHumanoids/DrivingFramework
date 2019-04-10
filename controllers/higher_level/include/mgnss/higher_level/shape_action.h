
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
ShapeAction(mgnss::higher_level::QpAggravated& task, mwoibn::hierarchical_control::tasks::ContactPoint& contact_point,
            std::vector<mwoibn::hierarchical_control::tasks::Angle>& steering, mgnss::higher_level::SteeringShape& steering_reference,
            mwoibn::hierarchical_control::tasks::Aggravated& aggravated, mwoibn::hierarchical_control::tasks::Aggravated& angles,
            mwoibn::hierarchical_control::tasks::Aggravated& caster, mwoibn::hierarchical_control::tasks::Aggravated& camber,
             mgnss::higher_level::StateMachine& state_machine, mgnss::higher_level::QrTask& unconstrainted,
            hierarchical_control::State& state, mwoibn::Vector3& next_step, double dt, mwoibn::robot_class::Robot& robot) :
            Primary(task, state.memory), _state(state), _qr_task(task), _contact_point(contact_point), _steering(steering),
            _steering_reference(steering_reference), _aggravated(aggravated), _angles(angles), _caster(caster), _camber(camber),
             _state_machine(state_machine), _unconstrainted(unconstrainted), _dt(dt), _next_step(next_step), _robot(robot){

               _desired_steer.setZero(4);
               _modified_support.setZero(8);
               _support_world.setZero(8);
               _support.setZero(12);
               _eigen_scalar.setZero(1);

               _robot.command.add(QR_TASK_VELOCITY, _robot.getDofs());
               _robot.command.add("QR_TASK_POSITION", _robot.getDofs());
               _robot.command.add("QR_TASK_TORQUE", _robot.getDofs());
               _robot.command.add("QR_TASK_ACCELERATION", _robot.getDofs());

               temp_des.setZero(12);
               temp_qr.setZero(12);
}


virtual void run(){
    _angles.update();
    _state_machine.update();
    _qr_task._update();
    _qr_task.solve();
    // _unconstrainted.solve();
    // std::cout << "previous\t" << _state.command.transpose() << std::endl;

    // std::cout << "_qr_task\t" << _qr_task.raw().transpose() << std::endl;
    // std::cout << "_cost\t" << _qr_task.optimalCost() << std::endl;

    _robot.command[QR_TASK_VELOCITY].set(_qr_task.raw().head(_qr_task.activeDofs().size()), _qr_task.activeDofs());

    if(std::isinf(_qr_task.optimalCost())){
        ++infs;
        _robot.command[QR_TASK_VELOCITY].set(mwoibn::VectorN::Zero(_robot.command[QR_TASK_VELOCITY].size()));
        std::cerr << infs << "\t" << _qr_task.optimalCost() << std::endl;
    }
    // std::cout << "_quadratic\n" << _qr_task.cost().quadratic << std::endl;
    // std::cout << "_linear\t" << _qr_task.cost().linear.transpose() << std::endl;

    // std::cout << "_steering\t" << _steering[0].getJacobian().transpose() << std::endl;

    for(int i =0; i < 4; i++){
        _eigen_scalar.noalias() =  _steering[i].getJacobian()*_qr_task.raw().head(_steering[i].getJacobian().cols() ) ;
        _desired_steer[i] = -_eigen_scalar[0]*_dt;
    }

    // mwoibn::VectorN _modified_support  =  _state_machine.stateJacobian()*_qr_task.raw().head(_contact_point.getJacobian().cols());
    // mwoibn::VectorN _modified_support(8);
    _qr_task.task(0).set(_qr_task.raw());
    _qr_task.task(0).transform();
    _support_world.noalias()  =  _state_machine.stateJacobian()*_qr_task.task(0).get();
    _support_world +=  _state_machine.stateOffset();

    for(int i =0; i < 4; i++){
      test__ = mwoibn::Vector3::Zero();
      test__.head<2>() = _support_world.segment<2>(2*i);
      _modified_support.segment<2>(2*i) = (_state_machine.steeringFrames()[i]->rotation*test__).head<2>();
    }
    // std::cout << "_modified_support\t" << _modified_support.transpose() << std::endl;
    // std::cout << "unconstrained_support\t" << _unconstrainted.get().transpose() << std::endl;

    // std::cout << "_desired_steer\t" << _desired_steer.transpose() << std::endl;
    // std::cout << "_caster\t" << (_caster.getJacobian()*_qr_task.raw().head(_caster.getJacobian().cols())).transpose() << std::endl;

    if(std::isinf(_qr_task.optimalCost())){
      _desired_steer.setZero();
      _modified_support.setZero();
    }


    //
    for(int i =0; i < 4; i++){
      _support.segment<3>(3*i) = _contact_point.getReferenceWorld(i);
      _support.segment<2>(3*i).noalias() += _modified_support.segment<2>(2*i)*_dt;
    //   // std::cout << i << "_modified_support\t" << (_modified_support.segment<2>(2*i)).transpose() << std::endl;
    //   // std::cout << i << "_modified_support\t" << (_modified_support.segment<2>(2*i)*_dt).transpose() << std::endl;
    //   // std::cout << i << "_support\t" << (_support.segment<3>(3*i)*_dt).transpose() << std::endl;
    }

    temp_qr = _support;
    //
    mwoibn::Vector3 support_i;
    for(int i =0; i < 4; i++){
      support_i = _support.segment<3>(3*i);
      _contact_point.setReferenceWorld(i, support_i, false);
    }

    _steering_reference.compute(_next_step, _desired_steer);
    //
    for(int i =0; i < 4; i++)
      _steering[i].setReference(_steering_reference.get()[i]);

    temp_des.setZero(12);
    for(int i =0; i < 4; i++){
      support_i = _support.segment<3>(3*i);
      support_i.head<2>() -= _modified_support.segment<2>(2*i)*_dt;
      // std::cout << i << "\t_modified_support\t" << (_modified_support.segment<2>(2*i)*_dt).transpose() << std::endl;
      // std::cout << i << "\tsupport_i\t" << support_i.transpose() << std::endl;

      vel__.setZero();
      vel__.head<2>() = _modified_support.segment<2>(2*i);

      test__.noalias() = _state_machine.steeringFrames()[i]->rotation.transpose()*vel__*_dt;
      // std::cout << "test\t" << test__.transpose() << std::endl;
      test__[1] = 0;
      support_i.noalias() += _state_machine.steeringFrames()[i]->rotation*test__;
      // std::cout << "test\t" << (_state_machine.steeringFrames()[i]->rotation*test__).transpose() << std::endl;
      // std::cout << i << "\tsupport_i\t" << support_i.transpose() << std::endl;
      temp_des.segment<3>(3*i) = support_i;

      _contact_point.setReferenceWorld(i, support_i, false);
    }

      // std::cout << "_steering_reference.get()\t" << _steering_reference.get().transpose() << std::endl;


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
      // std::cout << _camber.getTask(i).getJacobian()  <<"\t";

      // std::cout << weight  <<"\t";
      // std::cout << 0.6*(1-std::tanh( 0.2*std::pow(weight,3) ))  <<"\t";
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
  for(int i =0 ; i < 4; i++){
  logger.add("cp_des_"   + std::to_string(i) + "_x", temp_des[3*i] );
  logger.add("cp_des_"   + std::to_string(i) + "_y", temp_des[3*i+1] );
  logger.add("cp_qr_"   + std::to_string(i) + "_x", temp_qr[3*i] );
  logger.add("cp_qr_"   + std::to_string(i) + "_y", temp_qr[3*i+1] );
  }
}

protected:
  hierarchical_control::State& _state;
  mgnss::higher_level::QpAggravated& _qr_task;
  mwoibn::hierarchical_control::tasks::ContactPoint &_contact_point;
  std::vector<mwoibn::hierarchical_control::tasks::Angle> &_steering;
  mgnss::higher_level::SteeringShape& _steering_reference;
  mwoibn::hierarchical_control::tasks::Aggravated& _aggravated; // steering
  mwoibn::hierarchical_control::tasks::Aggravated &_angles, &_caster, &_camber;
  mgnss::higher_level::QrTask& _unconstrainted;
  mgnss::higher_level::StateMachine& _state_machine;
  mwoibn::VectorN _weight;
  mwoibn::VectorN _desired_steer, _modified_support, _support_world, _support;
  mwoibn::Vector3 test__, vel__;
  mwoibn::VectorN _eigen_scalar;
  double _dt;
  mwoibn::Vector3& _next_step;
  mwoibn::robot_class::Robot& _robot;
  int infs = 0;
  mwoibn::VectorN temp_qr, temp_des;
  const std::string QR_TASK_VELOCITY = "QR_TASK_VELOCITY";
};

}
} // namespace package
} // namespace library
#endif
