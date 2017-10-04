#ifndef REFERENCE_GENERATION_UTILS_H
#define REFERENCE_GENERATION_UTILS_H

#include <Eigen/Dense>
#include <MathGeoLib/Math/float3.h>
//#include <MathGeoLib/Algorithm/Random/LCG.h>
#include <MathGeoLib/Geometry/GeometryAll.h>
namespace mwoibn{
namespace reference_generation
{
namespace Utils
{

float3 toFloat3(Eigen::Vector3d vc); // these should be templates
Eigen::Vector3d fromFloat3(float3); // these should be templates


float constrainAngle(float x);

double constrainAngle180(double x);
}
} // namespace package
} // namespace library

#endif // REFERENCE_GENERATION_UTILS_H
