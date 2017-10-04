#ifndef POINT_HANDLING_ROBOT_POINTS_HANDLER_H
#define POINT_HANDLING_ROBOT_POINTS_HANDLER_H

#include "mwoibn/robot_class/robot.h"
#include "mwoibn/point_handling/state_points_handler.h"
#include "mwoibn/point_handling/raw_positions_handler.h"
#include "mwoibn/point_handling/raw_orientations_handler.h"
#include "mwoibn/point_handling/raw_full_states_handler.h"

namespace mwoibn{

namespace point_handling
{
//typedef mwoibn::Vector3 State;

template <typename rawHandler, typename State> class RobotPointsHandler : public StatePointsHandler<rawHandler, State>
{
public:
  RobotPointsHandler(int chain_origin, mwoibn::robot_class::Robot& robot)
      : StatePointsHandler<rawHandler, State>(chain_origin, robot.getModel(), robot.state.state(robot_class::INTERFACE::POSITION))
  {
  }

  RobotPointsHandler(std::string chain_origin, mwoibn::robot_class::Robot& robot)
      : StatePointsHandler<rawHandler, State>(chain_origin, robot.getModel(), robot.state.state(robot_class::INTERFACE::POSITION))
  {
  }

  RobotPointsHandler(int chain_origin, mwoibn::robot_class::Robot& robot,
                     std::vector<Point> points)
      : StatePointsHandler<rawHandler, State>(chain_origin, robot.getModel(), robot.state.state(robot_class::INTERFACE::POSITION), points)
  {
  }

  RobotPointsHandler(std::string chain_origin, mwoibn::robot_class::Robot& robot,
                     std::vector<Point> points)
      : StatePointsHandler<rawHandler, State>(chain_origin, robot.getModel(), robot.state.state(robot_class::INTERFACE::POSITION), points)
  {
  }

  RobotPointsHandler(int chain_origin, mwoibn::robot_class::Robot& robot,
                     std::vector<int> reference_frames,
                     std::vector<State> states = {},
                     std::vector<std::string> names = {})
      : StatePointsHandler<rawHandler, State>(chain_origin, robot.getModel(), robot.state.state(robot_class::INTERFACE::POSITION), reference_frames,
                   states, names)
  {
  }

  RobotPointsHandler(std::string chain_origin, mwoibn::robot_class::Robot& robot,
                     std::vector<std::string> reference_frames,
                     std::vector<State> states = {},
                     std::vector<std::string> names = {})
      : StatePointsHandler<rawHandler, State>(chain_origin, robot.getModel(), robot.state.state(robot_class::INTERFACE::POSITION), reference_frames,
                   states, names)
  {
  }

  RobotPointsHandler(RobotPointsHandler& other)
      : StatePointsHandler<rawHandler, State>(other)
  {
  }

  RobotPointsHandler(RobotPointsHandler&& other)
      : StatePointsHandler<rawHandler, State>(std::move(other))
  {
  }
};

//specific implementations
class PositionsHandler: public RobotPointsHandler<RawPositionsHandler, Point::Position>{
public:

  using RobotPointsHandler::RobotPointsHandler;

  PositionsHandler(PositionsHandler&& other)
      : RobotPointsHandler<RawPositionsHandler, Point::Position>(std::move(other))
  {
  }
  PositionsHandler(PositionsHandler& other)
      : RobotPointsHandler<RawPositionsHandler, Point::Position>(other)
  {
  }
  ~PositionsHandler(){}
};

class OrientationsHandler: public RobotPointsHandler<RawOrientationsHandler, Point::Orientation>{
public:
  using RobotPointsHandler::RobotPointsHandler;
  OrientationsHandler(OrientationsHandler&& other)
      : RobotPointsHandler<RawOrientationsHandler, Point::Orientation>(std::move(other))
  {
  }
  OrientationsHandler(OrientationsHandler& other)
      : RobotPointsHandler<RawOrientationsHandler, Point::Orientation>(other)
  {
  }
  ~OrientationsHandler(){}
};
class FullStatesHandler: public RobotPointsHandler<RawFullStatesHandler, mwoibn::Vector7>{
public:
  using RobotPointsHandler::RobotPointsHandler;
  FullStatesHandler(FullStatesHandler&& other)
      : RobotPointsHandler<RawFullStatesHandler, mwoibn::Vector7>(std::move(other))
  {
  }
  FullStatesHandler(FullStatesHandler& other)
      : RobotPointsHandler<RawFullStatesHandler, mwoibn::Vector7>(other)
  {
  }
  ~FullStatesHandler(){}
};
} // namespace package
} // namespace library
#endif // ROBOT_POINTS_HANDLER_H


