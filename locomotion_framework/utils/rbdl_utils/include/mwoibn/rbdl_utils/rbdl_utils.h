#ifndef RBDL_UTILS_RBDL_UTILS_H
#define RBDL_UTILS_RBDL_UTILS_H

#include <rbdl/rbdl.h>

namespace mwoibn
{

namespace rbdl_utils
{

const std::string PACKAGE = "rbdl_utils";

//! Provides the SVD pseudoinverse for Eigen library
/**
 * \param[in] a the matrix pseudoinverse is computed for
 * \param[in] epsilon dafines numerical resultion of considered values, elements
 *of a matrix below this value are considered zero in copmutation of an inverse
 * \return a computed pseudoinverse
 *
 */
static RigidBodyDynamics::Math::Quaternion
fromMatrix(const RigidBodyDynamics::Math::Matrix3d& rotation, double eps = 1e-4)
{
//  std::cout << "new\n";
  double trace = rotation(0,0) + rotation(1,1) + rotation(2,2);
  if (trace > 0)
  {
//    std::cout << "w" << std::endl;
    double s = 2.0 * sqrt(trace + 1.0);
    return RigidBodyDynamics::Math::Quaternion(
        (-rotation(2,1) + rotation(1,2)) / s,
        (-rotation(0,2) + rotation(2,0)) / s,
        (-rotation(1,0) + rotation(0,1)) / s, 0.25 * s);
  }

  if (rotation(0,0) > rotation(1,1) && rotation(0,0) > rotation(2,2))
  {
//    std::cout << "x" << std::endl;

    double s = 2.0 * sqrt(1.0 + rotation(0,0) - rotation(1,1) - rotation(2,2));
    return RigidBodyDynamics::Math::Quaternion(
        0.25 * s, (-rotation(0,1) - rotation(1,0)) / s,
        (-rotation(0,2) - rotation(2,0)) / s,
        (-rotation(2,1) + rotation(1,2)) / s);
  }

  if (rotation(1,1) > rotation(2,2))
  {
//    std::cout << "y" << std::endl;

    double s =
        2.0 * sqrt(1.0 + rotation(1,1) - rotation(0,0) - rotation(2,2));
    return RigidBodyDynamics::Math::Quaternion(
        (-rotation(0,1) - rotation(1,0)) / s, 0.25 * s,
        (-rotation(1,2) - rotation(2,1)) / s,
        (-rotation(0,2) + rotation(2,0)) / s);
  }
//  std::cout << "z" << std::endl;

  double s = 2.0 * sqrt(1.0 + rotation(2,2) - rotation(0,0) - rotation(1,1));
  return RigidBodyDynamics::Math::Quaternion(
      (-rotation(0,2) - rotation(2,0)) / s,
      (-rotation(1,2) - rotation(2,1)) / s, 0.25 * s,
      (-rotation(1,0) + rotation(0,1)) / s);
}

} // namespace package
} // namespace library
#endif
