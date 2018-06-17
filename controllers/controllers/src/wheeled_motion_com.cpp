#include "mgnss/controllers/wheeled_motion_com.h"
#include "mgnss/controllers/steering_v2.h"
#include <mwoibn/hierarchical_control/tasks/cartesian_simplified_pelvis_task_v4.h>

mgnss::controllers::WheeledMotionCom::WheeledMotionCom(mwoibn::robot_class::Robot& robot)
        : WheelsControllerExtend(robot)
{
        _robot.wait();
        _robot.get();
        _robot.updateKinematics();
        _robot.centerOfMass().update();

        _createTasks();
        _initIK();

        mwoibn::VectorN init_steer;
        init_steer.setZero(4);
        _steering_ref_ptr.reset(new mgnss::events::Steering2(
                                        _robot, *_steering_ptr, init_steer, 0.7, 0.3, _robot.rate(), 0.05));

        _allocate();
        init();
}



void mgnss::controllers::WheeledMotionCom::_createTasks(){
        // Set-up hierachical controller
        _constraints_ptr.reset(
                new mwoibn::hierarchical_control::tasks::Constraints(_robot));
        mwoibn::Vector3 pelvis;
        pelvis << 0, 0, 1;
        mwoibn::point_handling::PositionsHandler pelvis_ph("ROOT", _robot,
                                                           _robot.getLinks("base"));
        _pelvis_position_ptr.reset(
                new mwoibn::hierarchical_control::tasks::CartesianSelective(pelvis_ph,
                                                                            pelvis));
        pelvis << 1, 1, 1;
        _pelvis_orientation_ptr.reset(
                new mwoibn::hierarchical_control::tasks::OrientationSelective(
                        mwoibn::point_handling::OrientationsHandler("ROOT", _robot,
                                                                    _robot.getLinks("base")),
                        pelvis, _robot));
        _com_ptr.reset(new mwoibn::hierarchical_control::tasks::CenterOfMass(_robot));
        _steering_ptr.reset(
                new mwoibn::hierarchical_control::tasks::CartesianFlatReferenceV4(
                        mwoibn::point_handling::PositionsHandler("ROOT", _robot,
                                                                 _robot.getLinks("wheels")),
                        _robot));

        _createAngleTasks();
}

void mgnss::controllers::WheeledMotionCom::_initIK(){

        int ratio = 1;
        double damp = 1e-4;

        _hierarchical_controller_ptr->addTask(*_constraints_ptr, 1.0, damp);
        _hierarchical_controller_ptr->addTask(_leg_steer, 15.0*ratio, damp);
        _hierarchical_controller_ptr->addTask(*_pelvis_orientation_ptr, 20.0*ratio, damp);
        _hierarchical_controller_ptr->addTask(*_com_ptr, 20.0*ratio, damp);
        _hierarchical_controller_ptr->addTask(*_pelvis_position_ptr, 20.0*ratio, damp);
        _hierarchical_controller_ptr->addTask(*_steering_ptr, 10.0*ratio, damp);
        _hierarchical_controller_ptr->addTask(_leg_camber, 15.0*ratio, 0.04);
        _hierarchical_controller_ptr->addTask(_leg_castor, 10.0*ratio, 0.1);
}

void mgnss::controllers::WheeledMotionCom::_setInitialConditions(){

        WheelsControllerExtend::_setInitialConditions();

        _orientation = (mwoibn::Quaternion::fromAxisAngle(_y, _steering_ptr->getState()[4]) * mwoibn::Quaternion::fromAxisAngle(_x, _steering_ptr->getState()[5]));

        _pelvis_orientation_ptr->setReference(0, _orientation);

        _position = _pelvis_position_ptr->points().getPointStateWorld(0);
        _position.head(2) = _robot.centerOfMass().get().head(2);
        _pelvis_position_ptr->setReference(_position);
        _com_ptr->setReference(_position.head(2));
        _heading = _steering_ptr->getState()[2];
}

void mgnss::controllers::WheeledMotionCom::fullUpdate(const mwoibn::VectorN& support)
{
        _robot.get();
        _robot.updateKinematics();

        setSupport(support);
        update();

        _robot.send();
        _robot.wait();
}
void mgnss::controllers::WheeledMotionCom::compute()
{
        mgnss::controllers::WheelsController::compute();
        _correct();
}

void mgnss::controllers::WheeledMotionCom::_correct(){
        mwoibn::VectorN test(4);
        _robot.command.get(test,_select_steer);

        for (int i = 0; i < 4; i++)
        {
                test[i] = (test[i] < _l_limits[i]) ? test[i] + mwoibn::PI : test[i];
                test[i] = (test[i] > _u_limits[i]) ? test[i] - mwoibn::PI : test[i];
        }
        _robot.command.set(test,_select_steer);
}
