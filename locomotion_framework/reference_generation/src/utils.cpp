#include "mwoibn/reference_generation/utils.h"

namespace mwoibn {

namespace reference_generation
{

float3 Utils::toFloat3(Eigen::Vector3d vc)
{
  return float3(vc[0], vc[1], vc[2]);
}

Eigen::Vector3d Utils::fromFloat3(float3 f)
{
  Eigen::Vector3d vc;
  vc << f[0], f[1], f[2];
  return vc;
}

float Utils::constrainAngle(float x)
{
  x = fmod(x, 2 * pi);
  if (x < 0)
    x += 2 * pi;
  return x;
}

double Utils::constrainAngle180(double x)
{
  x = fmod(x + pi, 2 * pi);
  if (x < 0)
    x += 2 * pi;
  return x - pi;
}
} // namespace package
} // namespace library
