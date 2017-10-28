#ifndef PROGRAM_STEERING_H
#define PROGRAM_STEERING_H

#include <mwoibn/robot_class/robot.h>
#include <mwoibn/hierarchical_control/cartesian_simplified_pelvis_task_v3.h>

namespace events
{

void limit(double b_ref, double& b)
{

  b -= 6.28318531 * std::floor((b + 3.14159265) /
                               6.28318531); // this should normalize to -pi;pi
  b_ref -= 6.28318531 * std::floor((b_ref + 3.14159265) / 6.28318531);

  //  b = (std::fabs(b - b_ref) < 1.57079633) ? b : (b - b_ref < 0)
  //                                                    ? b + 3.14159265
  //                                                    : b - 3.14159265;
  b = (std::fabs(b - b_ref) < 1.57) ? b : (b - b_ref < 0) ? b + 3.14159265
                                                          : b - 3.14159265;
}

void limit(const mwoibn::VectorN& b_ref, mwoibn::VectorN& b)
{
  for (int i = 0; i < b.size(); i++)
    limit(b_ref[i], b[i]);
}

void jointLimits(double& b, double max = 2.79252680)
{

  if (b > max) // this is a temporary solution until they are joint
               // limits implemented
    b -= 3.1415926;
  else if (b < -max) // this is a temporary solution until they are
                     // joint limits implemented
    b += 3.1415926;
}

void jointLimits(mwoibn::VectorN& b, double max = 2.79252680)
{

  for (int i = 0; i < b.size(); i++)
    jointLimits(b[i], max);
}

RigidBodyDynamics::Math::VectorNd
initICM(mwoibn::robot_class::Robot& robot,
        RigidBodyDynamics::Math::Vector3d next_step,
        mwoibn::hierarchical_control::CartesianSimplifiedPelvisTask& plane)
{ // for now assume next_step is x,y,theta, it could also be done through
  // Robot
  // instance
  plane.updateState();
  mwoibn::Vector3 pelvis_state = plane.getState().head(3);

  mwoibn::point_handling::PositionsHandler reference_points(
      "pelvis", robot,
      std::vector<std::string>{"wheel_1", "wheel_2", "wheel_3", "wheel_4"});

  // This will be quite rigid to a current implementation but for now I can
  // take
  // is as elements of a  robot state (from floating base state)

  RigidBodyDynamics::Math::VectorNd b_des(4);
  for (int i = 0; i < reference_points.size(); i++)
  {
    b_des[i] = std::atan(-std::sin(pelvis_state[2]) * next_step[0] +
                         std::cos(pelvis_state[2]) * next_step[1] +
                         plane.getPointStateReference(i)[0] * next_step[2] /
                             std::cos(pelvis_state[2]) * next_step[0] +
                         std::sin(pelvis_state[2]) * next_step[1] -
                         plane.getPointStateReference(i)[1] * next_step[2]) +
               pelvis_state[2];
  }

  return b_des;
}

// this computes information in a reference frame ? -- then the pelvis should be
// removed
void ICM(mwoibn::robot_class::Robot& robot,
         RigidBodyDynamics::Math::Vector3d next_step,
         mwoibn::hierarchical_control::CartesianSimplifiedPelvisTask& plane,
         RigidBodyDynamics::Math::VectorNd& b_des)
{ // for now assume next_step is x,y,theta, it could also be done through
  // Robot
  // instance and next position

  plane.updateState();
  mwoibn::Vector3 pelvis_state = plane.getState().head(3);
  mwoibn::point_handling::PositionsHandler reference_points(
      "pelvis", robot,
      std::vector<std::string>{"wheel_1", "wheel_2", "wheel_3", "wheel_4"});
  mwoibn::point_handling::PositionsHandler pelvis(
      "ROOT", robot, std::vector<std::string>{"pelvis"});

  double steering;

  for (int i = 0; i < reference_points.size(); i++)
  {

    steering =
        std::atan2(-std::sin(pelvis_state[2]) * next_step[0] +
                       std::cos(pelvis_state[2]) * next_step[1] +
                       plane.getPointStateReference(i)[0] * next_step[2],
                   std::cos(pelvis_state[2]) * next_step[0] +
                       std::sin(pelvis_state[2]) * next_step[1] -
                       plane.getPointStateReference(i)[1] * next_step[2]);

    b_des[i] -= pelvis_state[2];

    limit(b_des[i], steering);

    b_des[i] = steering + pelvis_state[2];
  }
}

double PT(mwoibn::hierarchical_control::CartesianSimplifiedPelvisTask& plane,
          double b_icm, int i, double margin)
{
  // Desired state

  mwoibn::VectorN error = plane.getWorldError().segment<2>(2 * i); // size 2

  double b_sp;

  if (std::fabs(error.norm() > margin))
  {
    b_icm -= plane.getState()[2];
    b_sp = std::atan2(error[1], error[0]);
    limit(b_icm, b_sp);
    b_sp += plane.getState()[2];
  }
  else
    b_sp = b_icm;

  return b_sp;
}

void SPT(mwoibn::hierarchical_control::CartesianSimplifiedPelvisTask& plane,
         const mwoibn::VectorN& b_icm, mwoibn::VectorN& b_sp, double margin)
{
  // Desired state
  for (int i = 0; i < b_sp.size(); i++)
  {
    b_sp[i] = PT(plane, b_icm[i], i, margin);
  }
}

// void combined(
//    mwoibn::robot_class::Robot& robot,
//    mwoibn::hierarchical_control::CartesianSimplifiedPelvisTask& plane,
//    mwoibn::VectorN& b_st, RigidBodyDynamics::Math::Vector3d next_step,
//    double K_icm = 0.05, double K_sp = 0.95, double margin = 0.01,
//    double max = 2.79252680)
//{

//  ICM(robot, next_step, plane, b_st); // b_st should be updated to be a
//  current
//                                      // ICM, with soultion closest to the
//                                      // previous one

//  mwoibn::VectorN b_sp(4);
//  SPT(plane, b_st, b_sp, margin); // b_sp should be a polygon regulation, with
//                                  // soultion closest to the ICM

//  b_st = (K_icm * b_st + K_sp * b_sp) / (K_icm + K_sp);

//  b_st = b_st - plane.getState()[2] * mwoibn::VectorN::Ones(4);
//  jointLimits(b_st, max);
//  b_st = b_st + plane.getState()[2] * mwoibn::VectorN::Ones(4);

//}

mwoibn::VectorN
ICM(mwoibn::Vector3 next_step,
    mwoibn::hierarchical_control::CartesianSimplifiedPelvisTask& plane,
    mwoibn::VectorN& v_icm)
{ // for now assume next_step is x,y,theta, it could also be done through
  // Robot
  // instance and next position

  plane.updateState();
  mwoibn::Vector3 pelvis_state = plane.getState().head(3);

  mwoibn::VectorN b_icm(plane.points().size());
  v_icm.resize(plane.points().size());

  for (int i = 0; i < plane.points().size(); i++)
  {
    double x = std::cos(pelvis_state[2]) * next_step[0] +
               std::sin(pelvis_state[2]) * next_step[1] -
               plane.getPointStateReference(i)[1] * next_step[2];
    double y = -std::sin(pelvis_state[2]) * next_step[0] +
               std::cos(pelvis_state[2]) * next_step[1] +
               plane.getPointStateReference(i)[0] * next_step[2];

    double t = (next_step[2] < 0) ? -1 : 1;
    b_icm[i] = std::atan2(y, x); // this uses atan2 to avoid
                                 // singularities in a atan
                                 // implementation
    v_icm[i] = x * std::cos(b_icm[i]) + y * std::sin(b_icm[i]);
  }

  return b_icm;
}

mwoibn::VectorN
ICM2(mwoibn::Vector3 next_step,
     mwoibn::hierarchical_control::CartesianSimplifiedPelvisTask& plane,
     mwoibn::VectorN& v_icm, mwoibn::robot_class::Robot& robot)
{ // for now assume next_step is x,y,theta, it could also be done through
  // Robot
  // instance and next position

  plane.updateError();
  mwoibn::Vector3 pelvis_state = plane.getState().head(3);

  mwoibn::VectorN b_icm(plane.points().size());
  v_icm.resize(plane.points().size());

  std::vector<int> contact = {0, 1, 3, 2};

  for (int i = 0; i < plane.points().size(); i++)
  {
    double x = std::cos(pelvis_state[2]) * next_step[0] +
               std::sin(pelvis_state[2]) * next_step[1] -
               (robot.contacts().contact(contact[i]).getPosition()[1] -
                pelvis_state[1]) *
                   next_step[2];
    double y = -std::sin(pelvis_state[2]) * next_step[0] +
               std::cos(pelvis_state[2]) * next_step[1] +
               (robot.contacts().contact(contact[i]).getPosition()[0] -
                pelvis_state[0]) *
                   next_step[2];

    double t = (next_step[2] < 0) ? -1 : 1;
    b_icm[i] = std::atan2(y, x); // this uses atan2 to avoid
                                 // singularities in a atan
                                 // implementation
    v_icm[i] = x * std::cos(b_icm[i]) + y * std::sin(b_icm[i]);
  }

  return b_icm;
}

double PT(mwoibn::hierarchical_control::CartesianSimplifiedPelvisTask& plane,
          int i, double& v, double dt)
{
  // Desired state

  mwoibn::VectorN error = plane.getWorldError().segment<2>(2 * i); // size 2

  double b_sp;

  b_sp = std::atan2(error[1], error[0]); // this should always give a positive
                                         // result? - I need a diagram for that
  v = error.norm() / dt;

  return b_sp;
}

mwoibn::VectorN
SPT(mwoibn::hierarchical_control::CartesianSimplifiedPelvisTask& plane,
    mwoibn::VectorN& v_sp, double dt)
{
  // Desired state
  v_sp.resize(plane.points().size());
  mwoibn::VectorN b_sp(plane.points().size());

  for (int i = 0; i < plane.points().size(); i++)
  {
    b_sp[i] = PT(plane, i, v_sp[i], dt);
  }

  return b_sp;
}

void combined2(
    mwoibn::robot_class::Robot& robot,
    mwoibn::hierarchical_control::CartesianSimplifiedPelvisTask& plane,
    mwoibn::VectorN& b_st, RigidBodyDynamics::Math::Vector3d next_step,
    double K_icm, double K_sp, double dt, double margin = 0.03,
    double max = 2.79252680)
{

  mwoibn::VectorN v_icm; // store a icm velocities
  mwoibn::VectorN b_icm =
      ICM(next_step, plane, v_icm); // returns a velue in a robot space

  mwoibn::VectorN v_sp; // store a icm velocities
  mwoibn::VectorN b_sp =
      SPT(plane, v_sp, dt); // returns a velue in a robot space

  mwoibn::VectorN b(v_icm.size());

  for (int i = 0; i < v_icm.size(); i++)
  {
    if (std::fabs(v_icm[i]) + std::fabs(v_sp[i]) < margin / dt)
    {
      //      std::cout << "stop\t" << i << "\t" << v_sp[i] << "\t" << v_icm[i]
      //                << "\tlimit\t" << margin / dt << std::endl;
      v_icm[i] = 0;
      v_sp[i] = 0;

      b[i] = b_st[i] - plane.getState()[2];
    }
    else
      b[i] = std::atan2(K_icm * v_icm[i] * std::sin(b_icm[i]) +
                            K_sp * v_sp[i] * std::sin(b_sp[i]),
                        K_icm * v_icm[i] * std::cos(b_icm[i]) +
                            K_sp * v_sp[i] * std::cos(b_sp[i]));
  }

  b_st -= plane.getState()[2] * mwoibn::VectorN::Ones(plane.points().size());

  //  std::cout << "original\n" <<  b << std::endl;

  limit(b_st, b); // ensure continuity
                  //  std::cout << "continous\n" <<  b << std::endl;

  mwoibn::VectorN state =
      robot.state.get(mwoibn::robot_class::INTERFACE::POSITION);
  std::vector<std::string> names = {"hip1_1",   "hip1_2",   "hip1_3",
                                    "hip1_4",   "ankle2_1", "ankle2_2",
                                    "ankle2_3", "ankle2_4"};

  for (int i = 0; i < 4; i++)
  {
    double wheel = state[robot.getDof(names[4 + i])[0]];

    if (std::fabs(wheel) > (max - 0.005))
    {
      if (std::fabs(b_st[i] - b[i]) < 1.57079633)
      {
        if (b[i] > 0)
          b[i] -= 3.1415926;
        else
          b[i] += 3.1415926;
      }
    }
  }

  b_st = b + plane.getState()[2] * mwoibn::VectorN::Ones(plane.points().size());
}

class Steering
{

public:
  Steering(mwoibn::robot_class::Robot& robot,
           mwoibn::hierarchical_control::CartesianSimplifiedPelvisTask& plane,
           double K_icm, double K_sp, double dt, double margin = 0.05,
           double max = 2.79252680)
      : _plane(plane), _K_icm(K_icm), _K_sp(K_sp), _dt(dt), _margin(margin),
        _max(max),
        _state(robot.state.state(mwoibn::robot_class::INTERFACE::POSITION))
  {
    _v_icm.setZero(_size);
    _b_icm.setZero(_size);
    _v_sp.setZero(_size);
    _b_sp.setZero(_size);
    _b.setZero(_size);
    _b_st.setZero(_size);
    _plane_ref.setZero(2);
    _temp.setZero(_size);

    std::vector<std::string> names = {"hip1_1",   "hip1_2",   "hip1_3",
                                      "hip1_4",   "ankle2_1", "ankle2_2",
                                      "ankle2_3", "ankle2_4"};

    _dofs.resize(_size);

    for (int i = 0; i < _size; i++)
      _dofs[i] = robot.getDof(names[_size + i])[0];
/*
    std::cout << "time\t"
              << "heading\t"
              << "next_x\t"
              << "next_y\t"
              << "next_th\t"
              << "des_x_1\t"
              << "des_y_1\t"
              << "b_st_1\t"
              << "new_1\t"
              << "b_icm_1\t"
              << "v_icm_1\t"
              << "b_sp_1\t"
              << "v_sp_1\t"
              << "factor_1\t"
              << "final_1\t"
              << "des_x_2\t"
              << "des_y_2\t"
              << "b_st_2\t"
              << "new_2\t"
              << "b_icm_2\t"
              << "v_icm_2\t"
              << "b_sp_2\t"
              << "v_sp_2\t"
              << "factor_2\t"
              << "final_2\t"
              << "des_x_3\t"
              << "des_y_3\t"
              << "b_st_3\t"
              << "new_3\t"
              << "b_icm_3\t"
              << "v_icm_3\t"
              << "b_sp_3\t"
              << "v_sp_3\t"
              << "factor_3\t"
              << "final_3\t"
              << "des_x_4\t"
              << "des_y_4\t"
              << "b_st_4\t"
              << "new_4\t"
              << "b_icm_4\t"
              << "v_icm_4\t"
              << "b_sp_4\t"
              << "v_sp_4\t"
              << "factor_4\t"
              << "final_4\t" << std::endl; */
  }

  ~Steering() {}

  const mwoibn::VectorN& get() { return _b_st; }

  void compute(const mwoibn::Vector3 next_step)
  {
    //      std::cout << next_step << std::endl;
    _plane.updateState();
    _heading = _plane.getState()[2];

    _ICM(next_step); // returns a velue in a robot space

    _SPT(); // returns a velue in a robot space

    double l = _margin / _dt;

    l = l * l * l;
/*
    std::cout << _heading << "\t";
    std::cout << next_step[0] << "\t";
    std::cout << next_step[1] << "\t";
    std::cout << next_step[2] << "\t";
*/
    for (int i = 0; i < _size; i++)
    {
      double vel = std::fabs(_v_icm[i] + _v_sp[i]);
//      std::cout << _plane.getReference(i)[0] << "\t";
//      std::cout << _plane.getReference(i)[1] << "\t";

      if (vel < _margin / _dt)
      {
        //_v_icm[i] = 0;
        //_v_sp[i] = 0;

        _b[i] = std::atan2(_K_icm * _v_icm[i] * std::sin(_b_icm[i]) +
                               _K_sp * _v_sp[i] * std::sin(_b_sp[i]),
                           _K_icm * _v_icm[i] * std::cos(_b_icm[i]) +
                               _K_sp * _v_sp[i] * std::cos(_b_sp[i]));

                limit(_b_st[i]-_heading, _b[i]);
                _temp[i] = _b[i] - (_b_st[i] - _heading);

//        std::cout << _b_st[i] << "\t" << _b[i] << "\t" << _b_icm[i] << "\t" << _v_icm[i]
//                  << "\t" << _b_sp[i] << "\t" << _v_sp[i] << "\t" << vel* vel* vel / l;

        _b[i] = _b_st[i] - _heading;

        _b[i] += vel*vel*vel/l*_temp[i];

      }
      else
      {
        _b[i] = std::atan2(_K_icm * _v_icm[i] * std::sin(_b_icm[i]) +
                               _K_sp * _v_sp[i] * std::sin(_b_sp[i]),
                           _K_icm * _v_icm[i] * std::cos(_b_icm[i]) +
                               _K_sp * _v_sp[i] * std::cos(_b_sp[i]));

//        std::cout << _b_st[i] << "\t" << _b[i] << "\t" << _b_icm[i] << "\t" << _v_icm[i]
//                  << "\t" << _b_sp[i] << "\t" << _v_sp[i] << "\t" << vel* vel* vel / l;
      }

//      std::cout << "\t" << _b[i] << "\t";

      _b_st[i] -= _heading;
    }
//    std::cout << std::endl;

    limit(_b_st, _b); // ensure continuity
                      //  std::cout << "continous\n" <<  b << std::endl;

    for (int i = 0; i < _size; i++)
    {


      if (std::fabs(_state[_dofs[i]]) > (_max - 0.005))
      {
        if (std::fabs(_b_st[i] - _b[i]) < 1.57079633)
        {
          if (_b[i] > 0)
            _b[i] -= 3.1415926;
          else
            _b[i] += 3.1415926;
        }
      }

      _b_st[i] = _b[i] + _heading;
    }
  }

protected:
  mwoibn::hierarchical_control::CartesianSimplifiedPelvisTask& _plane;
  double _dt, _margin, _max, _K_icm, _K_sp, _heading, _x, _y;
  mwoibn::VectorN _v_icm, _b_icm, _v_sp, _b_sp, _b, _b_st, _plane_ref, _temp;
  const mwoibn::VectorN& _state;
  mwoibn::VectorInt _dofs;
  int _size = 4;

  void _ICM(mwoibn::Vector3 next_step)
  {

    for (int i = 0; i < _size; i++)
    {
      _plane_ref.noalias() = _plane.getPointStateReference(i).head(2);

      _x = std::cos(_heading) * next_step[0];
      _x += std::sin(_heading) * next_step[1];
      _x -= _plane_ref[1] * next_step[2];
      _y = -std::sin(_heading) * next_step[0];
      _y += std::cos(_heading) * next_step[1];
      _y += _plane_ref[0] * next_step[2];

      _b_icm[i] = std::atan2(_y, _x); // this uses atan2 to avoid
                                      // singularities in a atan
                                      // implementation
      _v_icm[i] = _x * std::cos(_b_icm[i]);
      _v_icm[i] += _y * std::sin(_b_icm[i]);
    }
  }

  void _SPT()
  {
    for (int i = 0; i < _size; i++)
      _PT(i);
  }

  void _PT(int i)
  {
    // Desired state

    _plane_ref[0] = _plane.getWorldError()[2 * i]; // size 2
    _plane_ref[1] = _plane.getWorldError()[2 * i + 1];

    _b_sp[i] = std::atan2(_plane_ref[1],
                          _plane_ref[0]);

    _v_sp[i] = _plane_ref.norm() / _dt;
  }
};
}

#endif // PROGRAM_STEERING_H
