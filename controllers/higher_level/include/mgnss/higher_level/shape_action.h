
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
            std::vector<mwoibn::hierarchical_control::tasks::SoftAngle>& steering, mgnss::higher_level::SteeringReference& steering_reference,
            mwoibn::hierarchical_control::tasks::Aggravated& aggravated, mwoibn::hierarchical_control::tasks::Aggravated& angles,
            mwoibn::hierarchical_control::tasks::Aggravated& caster, mwoibn::hierarchical_control::tasks::Aggravated& camber,
             mgnss::higher_level::StateMachine& state_machine,
            hierarchical_control::State& state, mwoibn::Vector3& next_step, double dt, mwoibn::robot_class::Robot& robot) :
            Primary(task, state.memory), _qr_task(task), _contact_point(contact_point), _steering(steering),
            _steering_reference(steering_reference), _aggravated(aggravated), _angles(angles), _caster(caster), _camber(camber),
             _state_machine(state_machine), _dt(dt), _next_step(next_step), _robot(robot){

               _desired_steer.setZero(4);
               _current_steer.setZero(4);
               _modified_support.setZero(12);
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


//    now = std::chrono::high_resolution_clock::now();
    _qr_task.solve();
//    end = std::chrono::high_resolution_clock::now();
//    elapsed_solve = end - now;

    _robot.states[QR].velocity.set(_qr_task.raw().head(_qr_task.activeDofs().size()), _qr_task.activeDofs());

    if(std::isinf(_qr_task.optimalCost())){
        ++infs;
        _robot.states[QR].velocity.set(mwoibn::VectorN::Zero(_robot.states[QR].velocity.size()));
        std::cerr << "inf\t" << infs << std::endl;
    }

    for(int i =0; i < 4; i++){
        _eigen_scalar.noalias() =  _steering[i].getJacobian()*_robot.states[QR].velocity.get();
        // _desired_steer[i] = -2*_eigen_scalar[0]*_dt;
        _desired_steer[i] = -_eigen_scalar[0];
        // _current_steer[i] = _steering[i].getCurrent();
        _current_steer[i] = _steering[i].getReference();

    }


    // mwoibn::VectorN _modified_support  =  _state_machine.cost_I.jacobian.get()*_qr_task.raw().head(_contact_point.getJacobian().cols());
    // mwoibn::VectorN _modified_support(8);

    _qr_task.task(0).set(_robot.states[QR].velocity.get());
    _qr_task.task(0).transform(); //?
    // std::cout << "wheel velocity\t" << _qr_task.task(0).get().transpose() << std::endl;
    // std::cout << "desired wheel velocity\t" << _qr_task.task(0).get().transpose() << std::endl;
    _support_world.noalias()  =  _state_machine.cost_I.jacobian.get()*_qr_task.task(0).get().head<12>();
    // std::cout << "jacobian\t" << _support_world.transpose() << std::endl;
    _support_world +=  _state_machine.cost_I.offset.get().head<8>();

    // std::cout << "offset\t" <<  _state_machine.cost_I.offset.get().head<8>().transpose() << std::endl;

    // std::cout << "solution\t" <<  _support_world.transpose() << std::endl;
    //
    // for(int i =0; i < 4; i++){
    //   test__ = mwoibn::Vector3::Zero();
    //   test__.head<2>() = _support_world.segment<2>(2*i);
    //   _modified_support.segment<2>(2*i) = (_state_machine.steeringFrames()[i]->rotation*test__).head<2>();
    // }

    if(std::isinf(_qr_task.optimalCost())){
      _desired_steer.setZero();
      _modified_support.setZero();
      _support_world.setZero();
      // for(auto& constraint: _qr_task.hard_inequality)
      //   std::cout << constraint->getState().transpose() << std::endl;
    }

    // if(_desired_steer.norm() > 0.02){

    for(int i =0; i < 4; i++){
        // add saturation on the steering task up to
//        double temp_ = _steering[i].getCurrent() - (_current_steer[i] + _desired_steer[i]*_robot.rate());
//        mwoibn::eigen_utils::limitToHalfPi(temp_);
//        if(std::fabs(temp_) < 0.3)
            _steering[i].setReference(_current_steer[i] + _desired_steer[i]*_robot.rate());
//       else if(_steering[i].getCurrent() < (_current_steer[i] + _desired_steer[i]*_robot.rate() )
//            _steering[i].setReference(_steering[i].getCurrent() + 0.1*_robot.rate())

      // _steering[i].setReference(_current_steer[i]);
      // _eigen_scalar[0] = _desired_steer[i];
      //_steering[i].setVelocity(_eigen_scalar);
      }

      //std::cout << "steer\t" << _desired_steer.transpose() << std::endl;
    // }

    mwoibn::Vector3 support_i;
    _modified_support.setZero();
    support_i.setZero();
      for(int i =0; i < 4; i++){

        support_i.setZero();
        support_i[0] = _support_world[2*i];
        support_i[1] = _support_world[2*i+1];

        // support_i.head<2>() = _support_world.segment<2>(2*i);

        test__.noalias() = _state_machine.steeringFrames()[i]->rotation.transpose()*support_i;
        test__[1] = 0;
        support_i = _state_machine.steeringFrames()[i]->rotation*test__;
        _modified_support.segment<3>(3*i) = _contact_point.getCurrentWorld(i);
        _modified_support.segment<2>(2*i) += _support_world.segment<2>(2*i)*_robot.rate();

        if(_support_world.segment<2>(2*i).norm() > 0.0001){

        // std::cout << "get before\t" << _contact_point.getReferenceWorld(i).transpose() << std::endl;
          vel__ = _contact_point.getReferenceWorld(i)+  support_i*_robot.rate();
//        double temp_ = _steering[i].getCurrent() - (_current_steer[i] + _desired_steer[i]*_robot.rate());
//        mwoibn::eigen_utils::limitToHalfPi(temp_);
//        if(std::fabs(temp_) < 0.3)
          _contact_point.setReferenceWorld(i, vel__, false);
        // _contact_point.setReferenceWorld(i,_contact_point.getReferenceWorld(i), false);
        // std::cout << "vel__\t" << _contact_point.getReferenceWorld(i).transpose() << std::endl;
      }
    }
    // _contact_point.setVelocity(_modified_support);
    //
    // std::cout << "_support world\t" << _support_world.transpose() << std::endl;
    // std::cout << "_modified_support\t" << _modified_support.transpose() << std::endl;

//       _caster.update();
//
// // THIS SHOULD BE UPDATED LATER IN THE STACK
//     _contact_point.update();
//     _aggravated.update(); // steering

}

virtual void release(){ }

void log(  mwoibn::common::Logger& logger){
  logger.add("ref", 0  );

 for(int i =0 ; i < 4; i++){
   logger.add("cp_des_"   + std::to_string(i) + "_x", _contact_point.getReferenceWorld(i)[0] );
   logger.add("cp_des_"   + std::to_string(i) + "_y", _contact_point.getReferenceWorld(i)[1] );
   logger.add("cp_qr_"   + std::to_string(i) + "_x", _modified_support[3*i] );
   logger.add("cp_qr_"   + std::to_string(i) + "_y", _modified_support[3*i+1] );

   logger.add("cp_"   + std::to_string(i) + "_x", _contact_point.getCurrentWorld(i)[0] );
   logger.add("cp_"   + std::to_string(i) + "_y", _contact_point.getCurrentWorld(i)[1] );
//    logger.add("cp_qr_"   + std::to_string(i) + "_x", temp_qr[3*i] );
//    logger.add("cp_qr_"   + std::to_string(i) + "_y", temp_qr[3*i+1] );
   double temp_ = _steering[i].getCurrent() - (_current_steer[i] + _desired_steer[i]*_robot.rate());
   mwoibn::eigen_utils::limitToHalfPi(temp_);

   logger.add("is_steer_"   + std::to_string(i), temp_ );
   logger.add("steer_error_"   + std::to_string(i), _steering[i].getError()[0] );
   logger.add("caster_error_"   + std::to_string(i), _caster.getError()[i] );
   logger.add("camber_error_"   + std::to_string(i), _camber.getError()[i] );
//   logger.add("desired_steer_"   + std::to_string(i), _steering[i].getReference() );
 }
    if(std::isinf(_qr_task.optimalCost()))
        logger.add("cost_", -mwoibn::NON_EXISTING );
    else
        logger.add("cost_", _qr_task.optimalCost() );
  // logger.add("_modified_support",_modified_support.norm() );

//    logger.add("elapsed_solve", elapsed_solve.count()     );

  // for(int i = 0; i < _qr_task.hard_inequality.size(); i++)
  //   logger.add("inequality_"+std::to_string(i), _qr_task.hard_inequality.getState()[i]);

}

protected:
  // hierarchical_control::State& _state;
  mgnss::higher_level::QpAggravated& _qr_task;
  mwoibn::hierarchical_control::tasks::ContactPoint &_contact_point;
  std::vector<mwoibn::hierarchical_control::tasks::SoftAngle> &_steering;
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
