#ifndef EIGEN_UTILS_QUATERNION_H
#define EIGEN_UTILS_QUATERNION_H

#include <Eigen/Dense>
#include <utility>

namespace mwoibn
{

class Quaternion : public Eigen::Quaterniond
{

public:
  Quaternion(double x, double y, double z, double w, double normalize = true)
      : Eigen::Quaterniond(w, x, y, z)
  {
    if (!normalize)
      return;

    double n = norm();
    (*this).w() = (*this).w() / n;
    (*this).x() = (*this).x() / n;
    (*this).y() = (*this).y() / n;
    (*this).z() = (*this).z() / n;
  }

  Quaternion() : Eigen::Quaterniond(1, 0, 0, 0) {}

  Quaternion(const Quaternion& q) : Eigen::Quaterniond(q) {}

  Quaternion(const Quaternion&& q) : Eigen::Quaterniond(std::move(q)) {}

  Quaternion operator=(const Quaternion& q)
  {
    (*this).w() = q.w();
    (*this).x() = q.x();
    (*this).y() = q.y();
    (*this).z() = q.z();
  }

  Quaternion inverse()
  {
    return Quaternion((*this).x(), (*this).y(), (*this).z(), -(*this).w());
  }

  ~Quaternion() {}

  Quaternion operator*(const Quaternion& q) const
  {
    return Quaternion(w()*q.x() + x()*q.w() + y()*q.z() - z()*q.y(),
                      w()*q.y() - x()*q.z() + y()*q.w() + z()*q.x(),
                      w()*q.z() + x()*q.y() - y()*q.x() + z()*q.w(),
                      w()*q.w() - x()*q.x() - y()*q.y() - z()*q.z());
  }

  Quaternion chuj(const Quaternion& q) const
  {
    return Quaternion(q.w() * x() + q.x() * w() + q.y() * z() - q.z() * y(),
                      q.w() * y() + q.y() * w() + q.z() * x() - q.x() * z(),
                      q.w() * z() + q.z() * w() + q.x() * y() - q.y() * x(),
                      q.w() * w() - q.x() * x() - q.y() * y() - q.z() * z());
  }

  Quaternion operator*(const double& s) const
  {
    return Quaternion((*this).x() * s, (*this).y() * s, (*this).z() * s,
                      (*this).w() * s);
  }

  using Eigen::Quaterniond::w;
  using Eigen::Quaterniond::x;
  using Eigen::Quaterniond::y;
  using Eigen::Quaterniond::z;

  void transpose() { (*this).w() = -(*this).w(); }

  Quaternion transposed()
  {
    return Quaternion((*this).x(), (*this).y(), (*this).z(), -(*this).w());
  }

  friend std::ostream& operator<<(std::ostream& os, const Quaternion& q)
  {
    os << q.x() << '\t' << q.y() << '\t' << q.z() << '\t' << q.w() << '\n';
    return os;
  }

  Eigen::Vector3d axis()
  {
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

  /**
   * @brief toAxisAngle extracts rotation angle and angle from input quaternion
   * @param vector
   * @return
   */
  double toAxisAngle(Eigen::Vector3d& vector) const
  {
    return toAxisAngle(vector, *this);
  }

  static double toAxisAngle(Eigen::Vector3d& vector, const Quaternion quat)
  {
    Eigen::AngleAxisd angleAxis(quat);
    vector = angleAxis.axis();

    return angleAxis.angle();
  }

  Quaternion inversed() const
  {
    return Quaternion(-this->x(), -this->y(), -this->z(), this->w(), false);
  }

  Quaternion reciprocal() const
  {
    double n = this->norm();
    n = n * n;
    return Quaternion(-this->x() / n, -this->y() / n, -this->z() / n,
                      this->w() / n, false);
  }

  /**
   * @brief swingTwist
   * @param quat_swing
   * @return
   */
  Quaternion swingTwist(const Eigen::Vector3d dir, Quaternion& q_swing)
  {

    Quaternion q_twist = swingTwist(dir);

    q_swing = (*this)*q_twist.reciprocal();

    return q_twist;
  }

  Quaternion swingTwist(const Eigen::Vector3d dir)
  {

    double norm = dir.norm();
    norm = norm * norm;

    double u = dir.dot(this->axis());

    double m = norm * this->w();

    double l = std::sqrt(m * m + u * u * norm);

    Quaternion q_twist(dir[0] * u / l, dir[1] * u / l, dir[2] * u / l, m / l,
                       false);

    return q_twist;
  }

  Quaternion twistSwing(const Eigen::Vector3d dir)
  {

    return Quaternion(this->reciprocal()).swingTwist(dir).reciprocal();
  }

  Quaternion twistSwing(const Eigen::Vector3d dir, Quaternion& q_swing)
  {
    Quaternion q_twist = Quaternion(this->reciprocal()).swingTwist(dir, q_swing).reciprocal();
    q_swing = q_swing.reciprocal();

    return q_twist;
  }

  static Quaternion fromAxisAngle(const Eigen::Vector3d& vector, double angle)
  {
    return Quaternion(
        Eigen::Quaterniond(Eigen::AngleAxisd(angle, vector.normalized())));
  }

  using Eigen::Quaterniond::norm;

  static Quaternion fromMatrix(const Eigen::Matrix3d& mat)
  {
    return Quaternion(Eigen::Quaterniond(mat));
  }

  Eigen::Matrix3d toMatrix() const
  {
    return ((*this).toRotationMatrix());
  }

  Quaternion(Eigen::Quaterniond& q) : Eigen::Quaterniond(q) {}
  Quaternion(Eigen::Quaterniond&& q) : Eigen::Quaterniond(std::move(q)) {}

  template <typename Vector>
  static void toVector(const Quaternion& orientation, Vector& vector,
                       int first = 0)
  {
    vector[0 + first] = orientation.x();
    vector[1 + first] = orientation.y();
    vector[2 + first] = orientation.z();
    vector[3 + first] = orientation.w();
  }

  template <typename Vector>
  static void fromVector(Quaternion& orientation, const Vector& vector,
                         int first = 0)
  {
    orientation.x() = vector[0 + first];
    orientation.y() = vector[1 + first];
    orientation.z() = vector[2 + first];
    orientation.w() = vector[3 + first];
  }

  Eigen::Vector3d rotate (const Eigen::Vector3d &vec) const {

    Quaternion vec_quat (vec[0], vec[1], vec[2], 0.0), res_quat;

    //res_quat = vec_quat * (*this);
    //res_quat = reciprocal() * res_quat;

    res_quat = (*this)*vec_quat;
    res_quat = res_quat*reciprocal();

    return res_quat.axis();
  }
};
} // namespace

#endif // EIGEN_UTILS_QUATERNION_H
