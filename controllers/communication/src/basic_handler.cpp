#include "mgnss/communication/basic_handler.h"


events::BasicHandler::BasicHandler(
    mwoibn::hierarchical_control::ConstraintsTask& constraints,
    mwoibn::hierarchical_control::CenterOfMassTask& com,
    mwoibn::robot_class::Robot& robot
    )
    : _robot(robot), _constraints(constraints), _com(com)
{
  for (int i = 0; i < robot.contacts().size(); i++)
  {
    _chains.push_back(Chain(robot.contacts().contact(i)));
  }
  reset();
}

bool events::BasicHandler::circle(int i){
  // create com reference
  _robot.centerOfMass().update();

  Eigen::VectorXd original(3);
  original << _robot.centerOfMass().get().head(2), 0;

  Eigen::Vector3d one, two, three;
  double radious = i/100.0;
  one << -radious, 0, 0;
  two << radious, 0, 0;
  three << radious, radious, 0;

  _circle_ptr.reset(new mwoibn::reference_generation::Local_Circle(
                      original + one, original + two, original + three, 0.0005));

  _action = ACTION::CIRCLE;
}

bool events::BasicHandler::reset()
{
  _support_polygon.p.clear();
  for (auto chain : _chains)
  {
    if (chain.getReferenceState() == STATE::STATIC)
      _support_polygon.p.push_back(
          mwoibn::reference_generation::Utils::toFloat3(
              chain.getContactPosition()));
  }

  mwoibn::Vector3 msp_point =
      mwoibn::reference_generation::Utils::fromFloat3(
          _support_polygon.CenterPoint());
  _line_ptr.reset(new mwoibn::reference_generation::Line(_robot.centerOfMass().get().head(2), 0.0005, msp_point.head(2)));
  _action = ACTION::LINE;
  return true;
}

bool events::BasicHandler::stop()
{
  _line_ptr.reset(new mwoibn::reference_generation::Line(_robot.centerOfMass().get().head(2), 0.0005, _robot.centerOfMass().get().head(2)));

  _action = ACTION::LINE;
  stepReference();

  return true;
}

void events::BasicHandler::update()
{
  stepReference();
}


// TRAJECTORY GENERATOR
bool events::TrajectoryGenerator::circle(double r, double p2){

      Eigen::VectorXd original = _points.getPointStateWorld(0);

      original[2] = 0;
      Eigen::Vector3d one, three;

      one << r, p2, 0;
      three << r/2, p2/2, 0;

      _circle_ptr.reset(new mwoibn::reference_generation::Local_Circle(
                          original, original + one, original + three, 0.0005));

      _action = ACTION::CIRCLE;
  return true;
}


bool events::TrajectoryGenerator::line(double x, double y){

  Eigen::VectorXd origin = _points.getPointStateWorld(0).head(2);
  Eigen::VectorXd step(2);
  step << x, y;

  _line_ptr.reset(new mwoibn::reference_generation::Line(origin, 0.0005, origin+step));

  _action = ACTION::LINE;
  stepReference();
  return true;
}


void events::TrajectoryGenerator::update(){
  stepReference();
}


bool events::TrajectoryGenerator::stop()
{
  _line_ptr.reset(new mwoibn::reference_generation::Line(_points.getPointStateWorld(0).head(2), 0.0005, _points.getPointStateWorld(0).head(2)));

  _action = ACTION::LINE;
  stepReference();

  return true;
}
