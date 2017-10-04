#ifndef PROGRAMS_EVENT_HANDLER
#define PROGRAMS_EVENT_HANDLER

//#include <MathGeoLib/Geometry/GeometryAll.h>

//#include <mwoibn/robot_class/contact_v2.h>

//#include <mwoibn/robot_class/robot.h>
//#include <mwoibn/hierarchical_control/hierarchical_controller.h>
//#include <mwoibn/hierarchical_control/center_of_mass_task.h>
//#include <mwoibn/hierarchical_control/constraints_task.h>
#include <mwoibn/hierarchical_control/cartesian_selective_task.h>
#include <mwoibn/hierarchical_control/orientation_selective_task.h>

#include <mwoibn/point_handling/robot_points_handler.h>
// trajectory generation
//#include <mwoibn/reference_generation/line.h>
//#include <mwoibn/reference_generation/local_circle.h>
#include <mgnss/communication/basic_handler.h>

namespace events
{

class EventHandler
{
public:
  EventHandler(mwoibn::hierarchical_control::CartesianSelectiveTask& tasks,
               mwoibn::hierarchical_control::OrientationSelectiveTask& orientation,
               mwoibn::hierarchical_control::ConstraintsTask& constraints,
               mwoibn::hierarchical_control::CenterOfMassTask& com,
               mwoibn::point_handling::PositionsHandler& phs,
               mwoibn::hierarchical_control::HierarchicalController& controller,
               mwoibn::robot_class::Robot& robot);
  ~EventHandler() {}

  bool claim(int i);
  bool release(int i);
  bool raise(int i);
  bool put(int i);
  bool reset();
  void update();
  mwoibn::VectorBool getActiveContacts();
  std::vector<int> getReferenceContacts();
  bool condition();
  bool stop();
  bool circle(int i);
  bool regainContact(int i) { return _chains[i].setState(STATE::STATIC); }
  bool breakContact(int i) { return _chains[i].setState(STATE::SWING); }
  bool setAction(ACTION action)
  {
    if (_action == action)
      return false;
    _action = action;
    _mode = 0;
    return true;
  }
  void advance() { ++_mode; }

  RigidBodyDynamics::Math::VectorNd getReference()
  {
    if (_action == ACTION::LINE || (_action == ACTION::CIRCLE && _mode == 0))
      return _line_ptr->getCurrentPoint();
    if (_action == ACTION::CIRCLE && _mode == 1)
      return _circle_ptr->getCurrentPoint();
  }
  void stepReference()
  {
    if (_action == ACTION::LINE || (_action == ACTION::CIRCLE && _mode == 0))
      _com.setReference(_line_ptr->nextStep());
    else if (_action == ACTION::CIRCLE && _mode == 1)
    {
      if (_circle_ptr->isDone())
        _circle_ptr->setFinalState(_circle_ptr->getFinalState() + 3.14);

      _com.setReference(_circle_ptr->nextStep().head(2));
    }

    if (_action == ACTION::CIRCLE && _mode == 0) _circle_1();

  }

protected:
  math::Polygon _support_polygon;
  mwoibn::hierarchical_control::CartesianSelectiveTask& _tasks;
  mwoibn::hierarchical_control::OrientationSelectiveTask& _orientation;
  mwoibn::hierarchical_control::ConstraintsTask& _constraints;
  mwoibn::hierarchical_control::CenterOfMassTask& _com;
  mwoibn::point_handling::PositionsHandler& _phs;
  mwoibn::hierarchical_control::HierarchicalController& _controller;
  mwoibn::robot_class::Robot& _robot;
  std::unique_ptr<mwoibn::reference_generation::Line> _line_ptr;
  std::unique_ptr<mwoibn::reference_generation::Local_Circle> _circle_ptr;
  ACTION _action;

  std::vector<Chain> _chains;
  float _safety_boundry = 0.05;
  float eps = 0.01;
  void _claim_1(int i);
  void _claim_2(int i);
  void _put_1(int i);
  void _put_2(int i);
  void _put_3(int i);
  void _raise_1(int i);
  void _circle_1();
//  void _quternionHemisphere(RigidBodyDynamics::Math::Quaternion original,
//                           RigidBodyDynamics::Math::Quaternion& desired);

//  bool _quternionHemisphereOne(int i, RigidBodyDynamics::Math::Quaternion original,
//                              RigidBodyDynamics::Math::Quaternion& desired);


  int _mode;
};
}
#endif
