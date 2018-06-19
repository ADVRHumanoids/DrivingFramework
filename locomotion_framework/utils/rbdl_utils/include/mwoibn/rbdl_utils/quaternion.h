#ifndef RBDL_UTILS_QUATERNION_H
#define RBDL_UTILS_QUATERNION_H

#include <rbdl/rbdl.h>
#include <math.h>

namespace mwoibn
{

class Quaternion : protected RigidBodyDynamics::Math::Quaternion
{

public:
Quaternion(double x, double y, double z, double w)
        : RigidBodyDynamics::Math::Quaternion(x, y, z, w)
{
        double n = norm();
        (*this)[0] = (*this)[0]/n;
        (*this)[1] = (*this)[1]/n;
        (*this)[2] = (*this)[2]/n;
        (*this)[3] = (*this)[3]/n;


}
Quaternion() : RigidBodyDynamics::Math::Quaternion(0, 0, 0, 1) {
}

Quaternion(const Quaternion& q)
        : RigidBodyDynamics::Math::Quaternion(q[0], q[1], q[2], q[3])
{
}
Quaternion(const Quaternion&& q)
        : RigidBodyDynamics::Math::Quaternion(q[0], q[1], q[2], q[3])
{
}

Quaternion operator=(const Quaternion& q)
{
        (*this)[0] = q[0];
        (*this)[1] = q[1];
        (*this)[2] = q[2];
        (*this)[3] = q[3];
}

void transpose(){
        (*this)[3] = -q[3];
}

Quaternion transposed(){
        return Quaternion((*this).x(), (*this).y(), (*this).z(), -(*this).w());
}

double norm()
{
        double n = (*this)[0]*(*this)[0]+(*this)[1]*(*this)[1]+(*this)[2]*(*this)[2]+(*this)[3]*(*this)[3];
        return std::sqrt(n);
}

Quaternion operator*(const Quaternion& q) const
{
        return Quaternion(q[3] * (*this)[0] + q[0] * (*this)[3] +
                          q[1] * (*this)[2] - q[2] * (*this)[1],
                          q[3] * (*this)[1] + q[1] * (*this)[3] +
                          q[2] * (*this)[0] - q[0] * (*this)[2],
                          q[3] * (*this)[2] + q[2] * (*this)[3] +
                          q[0] * (*this)[1] - q[1] * (*this)[0],
                          q[3] * (*this)[3] - q[0] * (*this)[0] -
                          q[1] * (*this)[1] - q[2] * (*this)[2]);
}

Quaternion operator*(const double& s) const
{
        return Quaternion (
                       (*this)[0] * s,
                       (*this)[1] * s,
                       (*this)[2] * s,
                       (*this)[3] * s
                       );
}

friend std::ostream& operator<<(std::ostream& os, const Quaternion& q)
{
        os << q.x() << '\t' << q.y() << '\t' << q.z() << '\t' << q.w() << '\n';
        return os;
}

virtual ~Quaternion() {
}

double w() const {
        return (*this)[3];
}
double x() const {
        return (*this)[0];
}
double y() const {
        return (*this)[1];
}
double z() const {
        return (*this)[2];
}

RigidBodyDynamics::Math::Vector3d axis()
{
        RigidBodyDynamics::Math::Vector3d axis;
        axis << x(), y(), z();
        return axis;
}

bool checkHemisphere(mwoibn::Quaternion& other)
{
        return (((*this)[0] * other[0] + (*this)[1] * other[1] +
                 (*this)[2] * other[2] + (*this)[3] * other[3]) > 0);
}

void ensureHemisphere(mwoibn::Quaternion& other)
{

        if (checkHemisphere(other))
                return;

        other[0] = -other[0];
        other[1] = -other[1];
        other[2] = -other[2];
        other[3] = -other[3];
}

static Quaternion
fromAxisAngle(const RigidBodyDynamics::Math::Vector3d& vector, double angle)
{
        return Quaternion(
                       RigidBodyDynamics::Math::Quaternion::fromAxisAngle(vector, angle));
}

static Quaternion fromMatrix(const RigidBodyDynamics::Math::Matrix3d& mat)
{

        return Quaternion(RigidBodyDynamics::Math::Quaternion::fromMatrix(mat));
}

RigidBodyDynamics::Math::Matrix3d toMatrix() const
{
        return RigidBodyDynamics::Math::Quaternion::toMatrix();
}

Quaternion(RigidBodyDynamics::Math::Quaternion& q)
        : RigidBodyDynamics::Math::Quaternion(q)
{
}
Quaternion(RigidBodyDynamics::Math::Quaternion&& q)
        : RigidBodyDynamics::Math::Quaternion(std::move(q))
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

#endif // RBDL_UTILS_QUATERNION_H
