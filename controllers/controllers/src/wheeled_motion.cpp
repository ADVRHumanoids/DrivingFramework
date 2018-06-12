#include <mgnss/controllers/wheeled_motion.h>
#include <mgnss/controllers/steering_v4.h>
#include <mwoibn/hierarchical_control/tasks/cartesian_simplified_pelvis_task_v3.h>

mgnss::controllers::WheeledMotion::WheeledMotion(mwoibn::robot_class::Robot& robot)
        : mgnss::controllers::WheelsController(robot)
{

        _robot.wait();
        _robot.get();
        _robot.updateKinematics();

        _createTasks();
        _initIK();

        mwoibn::VectorN init_steerings;
        init_steerings.setZero(4);

        _steering_ref_ptr.reset(
                new mgnss::events::Steering4(_robot, *_steering_ptr, init_steerings, 0.7, 0.3, _dt, 0.1));

        _allocate();
        init();
}

void mgnss::controllers::WheeledMotion::_setInitialConditions(){

        _steering_ptr->init();
        _support.noalias() = _steering_ptr->getReference();
        _support_vel.setZero();

        for (int i = 0; i < _leg_z_ptr->points().size(); i++)
        {
                steerings[i] = 0;
                _leg_xy_ptr->setReference(i,
                                          _leg_xy_ptr->getOffset(i)*(
                                                  mwoibn::Quaternion::fromAxisAngle(_z, 0.0)));
        }

        _position = _pelvis_position_ptr->points().getPointStateWorld(0);
        _pelvis_position_ptr->setReference(_position);
        _heading = _steering_ptr->getState()[2];
}


void mgnss::controllers::WheeledMotion::_createTasks(){

        _constraints_ptr.reset(
                new mwoibn::hierarchical_control::tasks::Constraints(_robot));
        mwoibn::Vector3 pelvis;
        pelvis << 1, 1, 1;
        mwoibn::point_handling::PositionsHandler pelvis_ph("ROOT", _robot,
                                                           _robot.getLinks("base"));
        _pelvis_position_ptr.reset(
                new mwoibn::hierarchical_control::tasks::CartesianSelective(pelvis_ph,
                                                                            pelvis));
        _pelvis_orientation_ptr.reset(
                new mwoibn::hierarchical_control::tasks::OrientationSelective(
                        mwoibn::point_handling::OrientationsHandler("ROOT", _robot,
                                                                    _robot.getLinks("base")),
                        pelvis, _robot));
        _steering_ptr.reset(
                new mwoibn::hierarchical_control::tasks::CartesianSimplifiedPelvis(
                        mwoibn::point_handling::PositionsHandler("ROOT", _robot,
                                                                 _robot.getLinks("wheels")),
                        _robot));

        mwoibn::VectorN selection(12);
        selection << 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0;

        _leg_z_ptr.reset(new mwoibn::hierarchical_control::tasks::OrientationSelective(
                                 mwoibn::point_handling::OrientationsHandler("ROOT", _robot,
                                                                             _robot.getLinks("camber")),
                                 selection, _robot));

        selection << 1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0, 1.0, 1.0, 0.0;

        _leg_xy_ptr.reset(new mwoibn::hierarchical_control::tasks::OrientationSelective(
                                  mwoibn::point_handling::OrientationsHandler("ROOT", _robot,
                                                                              _robot.getLinks("camber")),
                                  selection, _robot));

}

void mgnss::controllers::WheeledMotion::_initIK(){

        int ratio = 1;
        double damp = 1e-4;
        // Set initaial HC tasks
        _hierarchical_controller_ptr->addTask(*_constraints_ptr, 1.0, damp);
        _hierarchical_controller_ptr->addTask(*_leg_z_ptr, 25.0 * ratio, damp);
        _hierarchical_controller_ptr->addTask(*_pelvis_orientation_ptr, 20.0 * ratio, damp);
        _hierarchical_controller_ptr->addTask(*_pelvis_position_ptr, 20.0 * ratio, damp);
        _hierarchical_controller_ptr->addTask(*_steering_ptr, 10.0 * ratio, damp);
        _hierarchical_controller_ptr->addTask(*_leg_xy_ptr, 50.0 * ratio, 0.1);

        _dt = _robot.rate();

}

void mgnss::controllers::WheeledMotion::resetSteering()
{
        for (int i = 0; i < 4; i++)
        {
                _leg_z_ptr->setReference(i, _leg_z_ptr->getOffset(i)*(
                                                 mwoibn::Quaternion::fromAxisAngle(_z, 0)));
        }
}

void mgnss::controllers::WheeledMotion::update(const mwoibn::VectorN& support,
                                               const mwoibn::Vector3& velocity,
                                               const double omega)
{
        _linear_vel = velocity;
        _angular_vel[2] = omega;

        setSupport(support);
        nextStep();
        compute();
}

void mgnss::controllers::WheeledMotion::fullUpdate(const mwoibn::VectorN& support)
{
        _robot.get();
        _robot.updateKinematics();

        setSupport(support);
        update();

        _robot.send();
        _robot.wait();
}

void mgnss::controllers::WheeledMotion::fullUpdate(const mwoibn::VectorN& support,
                                                   const mwoibn::Vector3& velocity,
                                                   const double omega)
{
        _linear_vel = velocity;
        _angular_vel[2] = omega;

        setSupport(support);
        update();
}

void mgnss::controllers::WheeledMotion::steering()
{

        mgnss::controllers::WheelsController::steering();

        _steering_ref_ptr->set(steerings);
}
