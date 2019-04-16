#ifndef __MGNSS_HIGHER_LEVEL_JOINT_CONSTRAINT_V2_H
#define __MGNSS_HIGHER_LEVEL_JOINT_CONSTRAINT_V2_H

#include "mwoibn/robot_class/robot.h"
#include "mgnss/higher_level/qp/constraints/constraint.h"
#include "mgnss/higher_level/qp/constraints/maximum_limit.h"
#include "mgnss/higher_level/qp/constraints/minimum_limit.h"
#include "mgnss/higher_level/qp/constraints/merge.h"
#include "mgnss/higher_level/qp/constraints/integrate.h"
#include "mgnss/higher_level/qp/constraints/diff.h"
#include <mwoibn/dynamic_models/basic_model.h>


namespace mgnss::higher_level::constraints
{

class JointConstraintV2: public Constraint{

  public:
    JointConstraintV2(mwoibn::robot_class::Robot& robot, const mwoibn::VectorInt& dofs, std::vector<mwoibn::Interface> interfaces, mwoibn::dynamic_models::BasicModel& gravity);
    JointConstraintV2(const JointConstraintV2& other);

    virtual void update();

  protected:
    mwoibn::robot_class::Robot& _robot;
    mwoibn::Matrix _support_jacobian;
    std::vector<mwoibn::Interface> _interfaces;
    mwoibn::VectorN _max_position, _max_velocity, _max_torque, _min_torque;
    mwoibn::dynamic_models::BasicModel& _gravity;
    mwoibn::VectorInt _inactive_dofs;
    mwoibn::VectorN _test_inactive;

    // std::vector<unsigned int>  _dofs;
    // std::vector<bool> _velocity_dofs, _position_dofs;

//    bool _velocity = false, _position = false;
    virtual JointConstraintV2* clone_impl() const override {return new JointConstraintV2(*this);}
    mgnss::higher_level::constraints::Merge _min, _max;
    std::unique_ptr<mgnss::higher_level::Constraint> _torque_max, _torque_min;

    void _initPositions(){
      // create constraing
      _min.add(mgnss::higher_level::constraints::Integrate(mgnss::higher_level::constraints::MinimumLimit(_support_jacobian, _robot.lower_limits["POSITION"].get()), _robot.rate(), _robot.state.position.get()));
      _max.add(mgnss::higher_level::constraints::Integrate(mgnss::higher_level::constraints::MaximumLimit(_support_jacobian, _robot.upper_limits["POSITION"].get()), _robot.rate(), _max_position));

      _init("POSITION", _min.end(0), _max.end(0));

    }

    void _initVelocities(){
      _min.add(mgnss::higher_level::constraints::MinimumLimit(_support_jacobian, _robot.lower_limits["VELOCITY"].get()));
      _max.add(mgnss::higher_level::constraints::MaximumLimit(_support_jacobian, _robot.upper_limits["VELOCITY"].get()));

      _init("VELOCITY", _min.end(0), _max.end(0));
    }

    void _initTorques(){
      _robot.lower_limits.add("BIAS_FORCE", _robot.getDofs());
      _robot.upper_limits.add("BIAS_FORCE", _robot.getDofs());
      _torque_min.reset(new mgnss::higher_level::constraints::Diff(mgnss::higher_level::constraints::MinimumLimit(_gravity.getInertia(), _robot.lower_limits["BIAS_FORCE"].get()), _robot.rate(), _min_torque));
      _torque_max.reset(new mgnss::higher_level::constraints::Diff(mgnss::higher_level::constraints::MaximumLimit(_gravity.getInertia(), _robot.upper_limits["BIAS_FORCE"].get()), _robot.rate(), _max_torque));

      _init("TORQUE", *_torque_min, *_torque_max);
    }

    void _init(mwoibn::Interface interface, mgnss::higher_level::Constraint& min, mgnss::higher_level::Constraint& max);
    void _init(std::vector<mwoibn::Interface>& interfaces);

};

}
#endif
