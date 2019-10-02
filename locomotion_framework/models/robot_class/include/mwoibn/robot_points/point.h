#ifndef __MWOIBN__ROBOT_POINTS__POINT_H
#define __MWOIBN__ROBOT_POINTS__POINT_H

#include "mwoibn/robot_class/robot_class.h"
#include "mwoibn/robot_points/rotation.h"

namespace mwoibn
{
namespace robot_points
{

// Forward declaration of the Rotation class
class Rotation;

//! This class provides a common interface to impelment different robot characteristics 
/** A robot point is defined as any element that can be defined by the vector value and a matrix mapping
 *
 */
class Point
{

public:
  //! Constructor
  /** @param[in] state - size of a state vector, number of rows in a mapping matrix
   *  @param[in] dofs - number of columns in a mapping matrix
   */
  Point(unsigned int state, unsigned int dofs){
    _jacobian.setZero(state, dofs);
    _point.setZero(state);
  }

  Point(const Point& other): _jacobian(other._jacobian), _point(other._point){
  }

  Point( Point&& other): _jacobian(other._jacobian), _point(other._point){
  }


  virtual ~Point() {}

  //! update a point state (vector)
  virtual void compute() = 0;

  //! update a point mapping (matrix)
  virtual void computeJacobian() = 0;

  //! update a robot point
  /** @param[in] jacobain - should the matrix be updated */
  virtual void update(bool jacobian = true);

  //! Returns the reference to the mapping matrix
  const mwoibn::Matrix& getJacobian() const {return _jacobian;}

  //! Returns the reference to the vector (value)
  const mwoibn::VectorN& get() const {return _point;}

  //! Returns the size of the value vector
  int size(){return _point.size();}

  //! Returns the number of rows in a matrix
  int rows() {return _jacobian.rows();}
  //! Returns the number of columns in a matrix
  int cols() {return _jacobian.cols();}

  virtual Point& operator=(const Point& other);

  Point& operator+(const Point& other);

  Point& operator-(const Point& other);

  Point& operator+=(const Point& other);

  Point& operator-=(const Point& other);

  //! multiply the matrix by a scalar factor
  void multiplyJacobian(double factor);

  //! apply the rotation matrix to the point
  void rotateFrom(mwoibn::robot_points::Rotation& rotation, bool jacobian = false);

  //! apply the inverse rotation matrix to the point 
  void rotateTo(mwoibn::robot_points::Rotation& rotation, bool jacobian = false);

protected:

  mwoibn::Matrix _jacobian;
  mwoibn::VectorN _point;

};

} // namespace package
} // namespace library

#endif // __MWOIBN__ROBOT_POINTS__POINT_H
