#include "mwoibn/reference_generation/local_circle.h"

namespace mwoibn
{
namespace reference_generation
{

Local_Circle::Local_Circle(Eigen::VectorXd x_0, Eigen::VectorXd x_des,
                           Eigen::Vector3d axis, float limit)
{
  Sphere sph = Sphere(Utils::toFloat3(x_0), Utils::toFloat3(x_des));
  axis.normalize();

  Plane plane(Utils::toFloat3(x_0), Utils::toFloat3(x_des),
              Utils::toFloat3(x_0) + Utils::toFloat3(axis * sph.r));

  //    std::cout << "x_0: " << x_0 << std::endl;
  //    std::cout << "x_des: " << x_des << std::endl;
  //    std::cout << "axis * sph.r: " << axis * sph.r << std::endl;
  _circle = sph.Intersect(plane);

  _init(x_0, x_des, axis, limit);
}

Local_Circle::Local_Circle(Eigen::VectorXd x_0, Eigen::VectorXd x_des,
                           Eigen::VectorXd x_pos, Eigen::Vector3d axis,
                           float limit)
{
  Sphere sph = Sphere(Utils::toFloat3(x_pos), (x_0 - x_pos).norm());

  //    std::cout << "1 x_des: " << x_des << std::endl;
  float3 x_des_c = sph.ClosestPoint(Utils::toFloat3(x_des));
  x_des << x_des_c.x, x_des_c.y, x_des_c.z;
  //    std::cout << "2 x_des: " << x_des << std::endl;

  Plane plane(Utils::toFloat3(x_0), Utils::toFloat3(x_des),
              Utils::toFloat3(x_0) + Utils::toFloat3(axis * sph.r));

  //    std::cout << "x_0: " << x_0 << std::endl;
  //    std::cout << "x_des: " << x_des << std::endl;
  //    std::cout << "axis * sph.r: " << x_0 +axis * sph.r << std::endl;

  _circle = sph.Intersect(plane);
  _init(x_0, x_des, axis, limit);
}

Eigen::VectorXd Local_Circle::nextStep()
{

  if (fabs(_current_state - _final_state) >= fabs(_step))
    _current_state += _step;
  else
    _current_state = _final_state;

  return getPoint(_current_state);
}

Eigen::VectorXd Local_Circle::backStep()
{
  if (fabs(_current_state - _origin_state) >= fabs(_step))
    _current_state -= _step;
  else
    _current_state = _origin_state;

  return getPoint(_current_state);
}

Eigen::VectorXd Local_Circle::getPoint(double alpha)
{
  float3 point = _circle.GetPoint(alpha);
  Eigen::Vector3d vec;
  vec << point.x, point.y, point.z;
  return vec;
}

Eigen::VectorXd Local_Circle::getFinalPoint() { return getPoint(_final_state); }

Eigen::VectorXd Local_Circle::getOriginPoint()
{
  return getPoint(_origin_state);
}

Eigen::VectorXd Local_Circle::getCurrentPoint()
{
  return getPoint(_current_state);
}
bool Local_Circle::_init(Eigen::VectorXd x_0, Eigen::VectorXd x_des,
                         Eigen::Vector3d axis, float limit)
{

  float temp = ((_circle.GetPoint(0) - _circle.pos)
                    .AngleBetween(Utils::toFloat3(x_0) - _circle.pos));

  _current_state = (_circle.GetPoint(temp).Equals(Utils::toFloat3(x_0), 1e-2))
                       ? temp
                       : -temp;

  temp = ((_circle.GetPoint(0) - _circle.pos)
              .AngleBetween(Utils::toFloat3(x_des) - _circle.pos));
  _final_state = (_circle.GetPoint(temp).Equals(Utils::toFloat3(x_des), 1e-2))
                     ? temp
                     : -temp;

  temp = ((_circle.GetPoint(0) - _circle.pos)
              .AngleBetween(Utils::toFloat3(axis * _circle.r)));
  _origin_state = (_circle.GetPoint(temp).Equals(
                      Utils::toFloat3(axis * _circle.r) + _circle.pos, 1e-2))
                      ? temp
                      : -temp;

  _origin_state = Utils::constrainAngle(_origin_state);
  _final_state = Utils::constrainAngle(_final_state);
  _current_state = Utils::constrainAngle(_current_state);

  if (Min(_origin_state, _final_state, _current_state) == _origin_state)
  {
    if (Max(_final_state, _current_state) == _final_state)
      _final_state -= 2 * pi;
    else
      _current_state -= 2 * pi;
  }
  else if (Max(_origin_state, _final_state, _current_state) == _origin_state)
  {
    if (Min(_final_state, _current_state) == _final_state)
      _final_state += 2 * pi;
    else
      _current_state += 2 * pi;
  }

  _step = acos(1 - limit * limit / _circle.r / _circle.r / 2); // orientation

  std::cout << limit << std::endl;
  std::cout << _step << std::endl;
  //    if (fabs(_step) < 1e-5) _step = 1e-5;

  if (Sign(_step) != Sign(_final_state - _origin_state))
  {
    _step = -_step;
  }

  return true;
}
} // namespace package
} // namespace library
