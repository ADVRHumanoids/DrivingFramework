#ifndef POINT_HANDLING_ROBOT_POINTS_HANDLER_H
#define POINT_HANDLING_ROBOT_POINTS_HANDLER_H

#include "mwoibn/robot_class/robot.h"
#include "mwoibn/point_handling/raw_positions_handler.h"
#include "mwoibn/point_handling/raw_orientations_handler.h"
#include "mwoibn/point_handling/raw_full_states_handler.h"

namespace mwoibn {

namespace point_handling
{
//typedef mwoibn::Vector3 State;

template <typename rawHandler, typename State>
class RobotPointsHandler : public rawHandler
{
public:
RobotPointsHandler(int chain_origin, mwoibn::robot_class::Robot& robot)
        : rawHandler(chain_origin, robot.getModel(), robot.state)
{
}

RobotPointsHandler(std::string chain_origin, mwoibn::robot_class::Robot& robot)
        : rawHandler(chain_origin, robot.getModel(), robot.state)
{
}

RobotPointsHandler(int chain_origin, mwoibn::robot_class::Robot& robot,
                   std::vector<Position> points)
        : rawHandler(chain_origin, robot.getModel(), robot.state, points)
{
}

RobotPointsHandler(std::string chain_origin, mwoibn::robot_class::Robot& robot,
                   std::vector<Position> points)
        : rawHandler(chain_origin, robot.getModel(), robot.state, points)
{
}

template <typename Type, typename Vector>
RobotPointsHandler(Type chain_origin, mwoibn::robot_class::Robot& robot,
                   Vector reference_frames,
                   std::vector<State> states = {
                   },
                   std::vector<std::string> names = {})
        : rawHandler(chain_origin, robot.getModel(), robot.state, reference_frames,
                                                states, names)
{
}

RobotPointsHandler(RobotPointsHandler& other)
        : rawHandler(other)
{
}

RobotPointsHandler(RobotPointsHandler&& other)
        : rawHandler(std::move(other))
{
}
};

//specific implementations
class PositionsHandler : public RobotPointsHandler<RawPositionsHandler, Point::Current>{
public:

using RobotPointsHandler::RobotPointsHandler;

PositionsHandler(PositionsHandler&& other)
        : RobotPointsHandler<RawPositionsHandler, Point::Current>(std::move(other))
{
}
PositionsHandler(PositionsHandler& other)
        : RobotPointsHandler<RawPositionsHandler, Point::Current>(other)
{
}
virtual ~PositionsHandler(){
}
};

class OrientationsHandler : public RobotPointsHandler<RawOrientationsHandler, Orientation::O>{
public:
using RobotPointsHandler::RobotPointsHandler;
OrientationsHandler(OrientationsHandler&& other)
        : RobotPointsHandler<RawOrientationsHandler, Orientation::O>(std::move(other))
{
}
OrientationsHandler(OrientationsHandler& other)
        : RobotPointsHandler<RawOrientationsHandler, Orientation::O>(other)
{
}
virtual ~OrientationsHandler(){
}
};
class FullStatesHandler : public RobotPointsHandler<RawFullStatesHandler, mwoibn::Vector7>{
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
virtual ~FullStatesHandler(){
}
};
} // namespace package
} // namespace library
#endif // ROBOT_POINTS_HANDLER_H
