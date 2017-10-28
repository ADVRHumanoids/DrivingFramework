#ifndef EIGEN_UTILS_QUATERNION_H
#define EIGEN_UTILS_QUATERNION_H

#include <Eigen/Dense>

namespace mwoibn
{

class Quaternion : public Eigen::Quaterniond
{

public:
  Quaternion(double x, double y, double z, double w)
      : Eigen::Quaterniond(w, x, y, z)
  {
    double n = norm();
    (*this).w() = (*this).w()/n;
    (*this).x() = (*this).x()/n;
    (*this).y() = (*this).y()/n;
    (*this).z() = (*this).z()/n;
  }

  Quaternion() : Eigen::Quaterniond(1, 0, 0, 0) {}

  Quaternion(const Quaternion& q)
      : Eigen::Quaterniond(q)
  {
  }

  Quaternion(const Quaternion&& q)
      : Eigen::Quaterniond(std::move(q))
  {
  }

  Quaternion operator= (const Quaternion& q) {
    (*this).w() = q.w();
    (*this).x() = q.x();
    (*this).y() = q.y();
    (*this).z() = q.z();

  }

  Quaternion inverse(){
    return Quaternion((*this).x(), (*this).y(), (*this).z(), -(*this).w());
  }

  ~Quaternion() {}

  Quaternion operator*(const Quaternion& q) const
  {
    return Quaternion(q.w() * (*this).x() + q.x() * (*this).w() +
                          q.y() * (*this).z() - q.z() * (*this).y(),
                      q.w() * (*this).y() + q.y() * (*this).w() +
                          q.z() * (*this).x() - q.x() * (*this).z(),
                      q.w() * (*this).z() + q.z() * (*this).w() +
                          q.x() * (*this).y() - q.y() * (*this).x(),
                      q.w() * (*this).w() - q.x() * (*this).x() -
                          q.y() * (*this).y() - q.z() * (*this).z());
  }

  Quaternion operator*(const double& s) const
  {
    return Quaternion (
                  (*this).x() * s,
                  (*this).y() * s,
                  (*this).z() * s,
                  (*this).w() * s
                  );
  }


  using Eigen::Quaterniond::w;
  using Eigen::Quaterniond::x;
  using Eigen::Quaterniond::y;
  using Eigen::Quaterniond::z;

  void transpose(){
    (*this).w() = -(*this).w();
  }

  Quaternion transposed(){
    return Quaternion((*this).x(), (*this).y(), (*this).z(), -(*this).w());
  }

  friend std::ostream &operator<<(std::ostream& os, const Quaternion& q){
        os << q.x() << '\t' << q.y() << '\t' <<  q.z() << '\t' << q.w()  << '\n' ;
        return os;
    }


  Eigen::Vector3d axis(){
    Eigen::Vector3d axis;
    axis << x(), y(), z();
    return axis;
  }

  bool checkHemisphere(mwoibn::Quaternion& other)
  {
    return ((*this).dot(other) > 0);
  }

  void ensureHemisphere(mwoibn::Quaternion& other)
  {

    if (checkHemisphere(other))
      return;

    other.x() = -other.x();
    other.y() = -other.y();
    other.z() = -other.z();
    other.w() = -other.w();
  }

  static Quaternion fromAxisAngle(const Eigen::Vector3d& vector, double angle)
  {

    return Quaternion(Eigen::Quaterniond(Eigen::AngleAxisd(angle, vector.normalized())));
  }

  using Eigen::Quaterniond::norm;


  static Quaternion fromMatrix(const Eigen::Matrix3d& mat)
  {
      return Quaternion(Eigen::Quaterniond(mat.transpose()));
  }

   Eigen::Matrix3d toMatrix() const
   {
     return ((*this).toRotationMatrix()).transpose();
   }

  Quaternion(Eigen::Quaterniond& q)
      : Eigen::Quaterniond(q)
  {
  }
  Quaternion(Eigen::Quaterniond&& q)
      : Eigen::Quaterniond(std::move(q))
  {
  }

  template <typename Vector>
  static void toVector(const Quaternion& orientation, Vector& vector, int first = 0){
    vector[0+first] = orientation.x();
    vector[1+first] = orientation.y();
    vector[2+first] = orientation.z();
    vector[3+first] = orientation.w();
  }

  template <typename Vector>
  static void fromVector(Quaternion& orientation, const Vector& vector, int first = 0){
    orientation.x() = vector[0+first];
    orientation.y() = vector[1+first];
    orientation.z() = vector[2+first];
    orientation.w() = vector[3+first];
  }

};
} // namespace


#endif // EIGEN_UTILS_QUATERNION_H
