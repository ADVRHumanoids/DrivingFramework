#ifndef PROGRAM_ODOMETRY_H
#define PROGRAM_ODOMETRY_H

#include <mwoibn/robot_class/robot.h>
#include <mwoibn/point_handling/robot_points_handler.h>


namespace mgnss {

namespace odometry {

/**
 * it assumes the pelvis orientation is provided by the external source? - in my case the imu? - do it using the online floating base feedback - remove the postion estimation
 */
class Odometry{
public:
    Odometry(mwoibn::robot_class::Robot& robot, std::vector<std::string> names, double r);
    ~Odometry(){}

    const mwoibn::Vector6& get(){return _base;}
    void update();


protected:
    mwoibn::robot_class::Robot& _robot; // robot is needed for the joint state and imu feedbacks

    mwoibn::VectorN _state, _previous_state, _error, _distance; // _state - current wheel position
    mwoibn::VectorInt _selector, _contacts; // _state - current wheel position

    std::vector<mwoibn::Point> _estimated, _pelvis; // _estimated - wheel center position
    std::vector<mwoibn::Vector3> _directions, _axes; // directions
    double _r;
    mwoibn::VectorInt _ids;
    mwoibn::Vector6 _base;

    mwoibn::point_handling::PositionsHandler _wheels_ph;

    void _compute1();
    void _compute2();
    void _compute3();

    void _average();
    void _distances();

    void _mad();

    int _max();
    int _min();

//    mwoibn::point_handling::FullStatesHandler _wheels_ph;
//    mwoibn::point_handling::OrientationsHandler _directions_ph;


};

}
}
#endif
