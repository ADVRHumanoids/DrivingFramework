#ifndef PROGRAMS_BASIC_HANDLER_H
#define PROGRAMS_BASIC_HANDLER_H

#include <MathGeoLib/Geometry/GeometryAll.h>

#include <mwoibn/robot_class/contact_v2.h>

#include <mwoibn/robot_class/robot.h>
#include <mwoibn/hierarchical_control/controllers/default.h>
#include <mwoibn/hierarchical_control/tasks/center_of_mass_task.h>
#include <mwoibn/hierarchical_control/tasks/constraints_task.h>
//#include <mwoibn/hierarchical_control/tasks/cartesian_selective_task.h>

#include <mwoibn/point_handling/robot_points_handler.h>
// trajectory generation
#include <mwoibn/reference_generation/line.h>
#include <mwoibn/reference_generation/local_circle.h>

namespace events {

enum class ACTION
{
RAISE,
CLAIM,
PUT,
RELEASE,
CIRCLE,
LINE,
NONE
};
enum class STATE
{
SWING,
STATIC
};

class Chain
{
public:
Chain(mwoibn::robot_class::ContactV2& contact)
        : _action(ACTION::NONE), _state(STATE::STATIC), _contact(contact),
        _mode(0), _reference_state(STATE::STATIC)
{
        _reference = _contact.getPosition().head(3);
        _reference[2] = -0.0001;
        //    if (!_contact.isActive() || _reference[2] > 0.005)
        //    {
        //      _action = ACTION::PUT;
        //      _state = STATE::SWING;
        //    }
}

virtual ~Chain() {
}

/**
 * @brief setAction sets new action for a chain
 * @param new_action
 * @return true if new action was properly set, false if new action
 *****contradicts current status or it is already set
 *
 */
bool setAction(ACTION new_action)
{

        if (new_action == _action)
                return false;

        switch (new_action)
        {
        case ACTION::CLAIM:
                if (_action != ACTION::NONE)
                        return false;
                if (_state != STATE::STATIC)
                        return false;
                break;
        case ACTION::RELEASE:
                if (_state != STATE::SWING)
                        return false;
                if (_action == ACTION::CLAIM)
                        return false;
                if (_action == ACTION::RAISE)
                        return false;
                break;
        case ACTION::RAISE:
                if (_state != STATE::SWING)
                        return false;
                if (_action == ACTION::CLAIM)
                        return false;
                if (_action == ACTION::RELEASE)
                        return false;
                break;
        case ACTION::PUT:
                if (_state != STATE::SWING)
                        return false;
                if (_action == ACTION::CLAIM)
                        return false;
                if (_action == ACTION::RELEASE)
                        return false;
                break;
        }
        _action = new_action;
        _mode = 0;
        return true;
}

bool setState(STATE new_state)
{
        if (new_state == _state)
        {
                std::cout << "Couldn't change a state" << std::endl;
                return false;
        }
        else
                _state = new_state;
        return true;
}

bool setReferenceState(STATE new_state)
{
        if (new_state == _reference_state)
        {
                std::cout << "Couldn't change a state" << std::endl;
                return false;
        }
        else
                _reference_state = new_state;
        return true;
}

ACTION getAction() {
        return _action;
}
int getMode() {
        return _mode;
}

STATE getState() {

        return _state;
}
STATE getReferenceState() {
        return _reference_state;
}

RigidBodyDynamics::Math::Vector3d getReference() {
        return _reference;
}
RigidBodyDynamics::Math::Vector3d getContactPosition()
{
        RigidBodyDynamics::Math::Vector3d point = _contact.getPosition().head(3);
        point[2] = 0;

        return point;
}

void advance() {
        ++_mode;
}

protected:
ACTION _action;
STATE _state;
STATE _reference_state;
int _mode;

RigidBodyDynamics::Math::Vector3d _reference;
mwoibn::robot_class::ContactV2& _contact;
};


class BasicHandler {

public:
BasicHandler(mwoibn::hierarchical_control::tasks::Constraints& constraints,
             mwoibn::hierarchical_control::tasks::CenterOfMass& com,
             mwoibn::robot_class::Robot& robot
             );

virtual ~BasicHandler(){
}

bool reset();
void update();
bool stop();
//  bool set();
bool circle(int i);

bool setReferenceState(int i, STATE state){
        _chains[i].setReferenceState(state);
}

RigidBodyDynamics::Math::VectorNd getReference()
{
        if (_action == ACTION::LINE)
                return _line_ptr->getCurrentPoint();
        if (_action == ACTION::CIRCLE)
                return _circle_ptr->getCurrentPoint();
}
void stepReference()
{
        if (_action == ACTION::LINE)
                _com.setReference(_line_ptr->nextStep());
        if (_action == ACTION::CIRCLE)
        {
                if (_circle_ptr->isDone())
                        _circle_ptr->setFinalState(_circle_ptr->getFinalState() + 3.14);

                _com.setReference(_circle_ptr->nextStep().head(2));
        }
}

protected:
math::Polygon _support_polygon;
mwoibn::hierarchical_control::tasks::Constraints& _constraints;
mwoibn::hierarchical_control::tasks::CenterOfMass& _com;
mwoibn::robot_class::Robot& _robot;
std::unique_ptr<mwoibn::reference_generation::Local_Circle> _circle_ptr;
std::unique_ptr<mwoibn::reference_generation::Line> _line_ptr;

ACTION _action;

std::vector<Chain> _chains;

float _safety_boundry = 0.05;
float eps = 0.01;

};

class TrajectoryGenerator {

public:
TrajectoryGenerator(mwoibn::point_handling::PositionsHandler& points) : _points(points){
        stop();

}
virtual ~TrajectoryGenerator(){
}

bool circle(double r, double p2);
bool line(double x, double y);
void update();
bool stop();

RigidBodyDynamics::Math::VectorNd getReference()
{
        RigidBodyDynamics::Math::VectorNd point = RigidBodyDynamics::Math::VectorNd::Zero(3);

        if (_action == ACTION::LINE)
                point.head(2) = _line_ptr->getCurrentPoint();
        else if (_action == ACTION::CIRCLE)
                point.head(2) = _circle_ptr->getCurrentPoint().head(2);

        return point;
}

void stepReference()
{
        if (_action == ACTION::LINE)
                _line_ptr->nextStep();
        if (_action == ACTION::CIRCLE)
        {
//      if (_circle_ptr->isDone())
//        _circle_ptr->setFinalState(_circle_ptr->getFinalState() + 3.14);

                _circle_ptr->nextStep();
        }
}

protected:
std::unique_ptr<mwoibn::reference_generation::Local_Circle> _circle_ptr;
std::unique_ptr<mwoibn::reference_generation::Line> _line_ptr;
ACTION _action;
mwoibn::point_handling::PositionsHandler& _points;
};

}

#endif // BASIC_HANDLER_H
