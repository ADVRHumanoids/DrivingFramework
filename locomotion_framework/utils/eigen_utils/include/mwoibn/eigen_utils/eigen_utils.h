#ifndef EIGEN_UTILS_EIGEN_UTILS_H
#define EIGEN_UTILS_EIGEN_UTILS_H

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <memory>
#include <iostream>
#include <cmath>
#include <vector>

namespace mwoibn
{

namespace eigen_utils
{

const std::string PACKAGE = "eigen_utils";

//! Provides the SVD pseudoinverse for Eigen library
/**
 * \param[in] a the matrix pseudoinverse is computed for
 * \param[in] epsilon dafines numerical resultion of considered values, elements
 *******************************of a matrix below this value are considered zero in copmutation of an inverse
 * \return a computed pseudoinverse
 *
 */
template <typename _Matrix_Type_>
_Matrix_Type_
pseudoInverse(const _Matrix_Type_& a,
              double epsilon = std::numeric_limits<double>::epsilon())
{
        Eigen::JacobiSVD<_Matrix_Type_> svd(a, Eigen::ComputeThinU |
                                            Eigen::ComputeThinV);
        double tolerance = epsilon * std::max(a.cols(), a.rows()) *
                           svd.singularValues().array().abs() (0);
        return svd.matrixV() *
               (svd.singularValues().array().abs() > tolerance)
               .select(svd.singularValues().array().inverse(), 0)
               .matrix()
               .asDiagonal() *
               svd.matrixU().adjoint();
}

template <typename Dynamic_Matrix> class PseudoInverse
{

public:
PseudoInverse(const Dynamic_Matrix matrix,
              double epsilon = std::numeric_limits<double>::epsilon())
{
        init(matrix, epsilon);
        compute(matrix);
}

PseudoInverse(PseudoInverse& other)
        : _tolerance(other._tolerance), _epsilon(other._epsilon),
        _inversedMatrix(other._inversedMatrix), _tempMatrix(other._tempMatrix)
{
        _svd_ptr.reset(new Eigen::JacobiSVD<Dynamic_Matrix>(
                               _inversedMatrix.cols(), _inversedMatrix.rows(),
                               Eigen::ComputeThinU | Eigen::ComputeThinV));
}

//  PseudoInverse(PseudoInverse&& other)
//      : _tolerance(other._tolerance), _epsilon(other._epsilon),
//        _inversedMatrix(other._inversedMatrix),
//        _tempMatrix(other._tempMatrix)
//  {
//    _svd_ptr.reset(new Eigen::JacobiSVD<Dynamic_Matrix>(
//        _inversedMatrix.cols(), _inversedMatrix.rows(),
//        Eigen::ComputeThinU | Eigen::ComputeThinV));
//  }

PseudoInverse() {
}

void compute(const Dynamic_Matrix& matrix)
{
        _svd_ptr->compute(matrix);

        _tolerance = _epsilon * _svd_ptr->singularValues().array().abs() (0);

        for (int i = 0; i < _dampedSingularValues.rows(); i++)
        {
                _dampedSingularValues(i, i) =
                        std::fabs(_svd_ptr->singularValues()[i] > _tolerance)
                        ? 1 / (_svd_ptr->singularValues()[i])
                        : 0;
        }

        _tempMatrix.noalias() = _svd_ptr->matrixV() * _dampedSingularValues;

        _inversedMatrix.noalias() = _tempMatrix * _svd_ptr->matrixU().adjoint();
}
void init(const Dynamic_Matrix matrix,
          double epsilon = std::numeric_limits<double>::epsilon())
{

        _svd_ptr.reset(new Eigen::JacobiSVD<Dynamic_Matrix>(
                               matrix.rows(), matrix.cols(),
                               Eigen::ComputeThinU | Eigen::ComputeThinV));

        _epsilon = epsilon * std::max(matrix.cols(), matrix.rows());
        _inversedMatrix = Dynamic_Matrix::Zero(matrix.cols(), matrix.rows());
        int size = std::min(matrix.cols(), matrix.rows());
        _tempMatrix = Dynamic_Matrix::Zero(matrix.cols(), size);
        _dampedSingularValues = Dynamic_Matrix::Zero(size, size);
}

const Dynamic_Matrix& get() {
        return _inversedMatrix;
}
virtual ~PseudoInverse() {
}

protected:
std::unique_ptr<Eigen::JacobiSVD<Dynamic_Matrix> > _svd_ptr;
double _tolerance, _epsilon;
Dynamic_Matrix _inversedMatrix, _tempMatrix, _dampedSingularValues;
};


template <typename Dynamic_Matrix, typename Scalar>
class Inverse
{
public:
Inverse(const Dynamic_Matrix& matrix,
               Scalar damping = std::numeric_limits<Scalar>::epsilon())
{
        init(matrix, damping);
        // compute(matrix);
}
Inverse(const Dynamic_Matrix& matrix,
               const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& damping)
{
        init(matrix, damping);
        // compute(matrix);
}

Inverse(
        const Inverse &other) : _inverse_ptr(std::move(other._inverse_ptr)), _damping(other._damping),  _inversed(other._inversed), _identity(other._identity)
{
}

Inverse(Inverse&& other) : _inverse_ptr(std::move(other._inverse_ptr)), _damping(other._damping), _inversed(other._inversed), _identity(other._identity)
{

}

Inverse() { }

void compute(const Dynamic_Matrix& matrix)
{
        _inversed = matrix;
        for (int i = 0; i < _inversed.rows(); i++)
                _inversed(i, i) += _damping[i];

        _inverse_ptr->compute(_inversed);
        _inversed.noalias() = _inverse_ptr->solve(_identity);
}

void init(const Dynamic_Matrix& matrix, Scalar damping){
        _damping.setConstant(std::min(matrix.rows(), matrix.cols()), damping);
        _init(matrix);
}

void init(const Dynamic_Matrix& matrix, const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& damping){
        if(damping.size() != std::min(matrix.rows(), matrix.cols()))
                throw(std::invalid_argument("Couldn't initialize pseudo inverse, incompatibile damping size"));
        _damping = damping;
        _init(matrix);
}

Scalar damping(int i){
        return _damping[i];
}

void setDamping(int i, double damp){
        _damping[i] = damp;
}

void setDamping(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& damp){
        for(int i = 0; i < _damping.size(); i++)
            _damping[i] = damp[i];
}

Eigen::Matrix<Scalar, Eigen::Dynamic, 1> damping() {return _damping;}


const Dynamic_Matrix& get() const {
        return _inversed;
}
virtual ~Inverse() {
}

protected:
std::unique_ptr<Eigen::LDLT<Dynamic_Matrix> > _inverse_ptr;
Eigen::Matrix<Scalar, Eigen::Dynamic, 1> _damping;
Dynamic_Matrix _inversed, _identity;

void _init(const Dynamic_Matrix& matrix)
{
      if(matrix.rows() != matrix.cols())
        throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": Can not initialize matrix inversed, expected square matrix."));

        _inversed.setZero(matrix.rows(), matrix.cols());
        _inverse_ptr.reset(new Eigen::LDLT<Dynamic_Matrix>(matrix.rows()));
        _damping = _damping.cwiseProduct(_damping);
        _identity.setIdentity(matrix.rows(), matrix.cols());
}
};




template <typename Dynamic_Matrix, typename Scalar> class PseudoInverse2
{

public:
PseudoInverse2(const Dynamic_Matrix& matrix,
               Scalar damping = std::numeric_limits<Scalar>::epsilon())
{
        init(matrix, damping);
        // compute(matrix);
}
PseudoInverse2(const Dynamic_Matrix& matrix,
               const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& damping)
{
        init(matrix, damping);
        // compute(matrix);
}

PseudoInverse2(
        const PseudoInverse2 &other) : _inverse_ptr(std::move(other._inverse_ptr)), _damping(other._damping),  _transposed(other._transposed), _inversed(other._inversed), _squared(other._squared), _identity(other._identity), _type(other._type)
{
}

PseudoInverse2(
        PseudoInverse2&& other) : _inverse_ptr(std::move(other._inverse_ptr)), _damping(other._damping),  _transposed(other._transposed), _inversed(other._inversed), _squared(other._squared), _identity(other._identity), _type(other._type)
{

}


PseudoInverse2() {
}

void compute(const Dynamic_Matrix& matrix)
{
        _transposed.noalias() = matrix.transpose();
        if(_type)
                _squared.noalias() = _transposed * matrix;
        else
                _squared.noalias() = matrix * _transposed;

        for (int i = 0; i < _squared.rows(); i++)
                _squared(i, i) += _damping[i];

        _inverse_ptr->compute(_squared);
        _squared.noalias() = _inverse_ptr->solve(_identity);

        if (_type)
                _inversed.noalias() = _squared * _transposed;
        else
                _inversed.noalias() = _transposed * _squared;

}
void init(const Dynamic_Matrix& matrix, Scalar damping){
        _damping.setConstant(std::min(matrix.rows(), matrix.cols()), damping);
        _init(matrix);
}

void init(const Dynamic_Matrix& matrix, const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& damping){
        if(damping.size() != std::min(matrix.rows(), matrix.cols()))
                throw(std::invalid_argument("Couldn't initialize pseudo inverse, incompatibile damping size"));
        _damping = damping;
        _init(matrix);
}

Scalar damping(int i){
        return _damping[i];
}

void setDamping(int i, double damp){
        _damping[i] = damp;
}

void setDamping(const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& damp){
        for(int i = 0; i < _damping.size(); i++)
            _damping[i] = damp[i];
}

Eigen::Matrix<Scalar, Eigen::Dynamic, 1> damping() {return _damping;}


const Dynamic_Matrix& get() const {
        return _inversed;
}
virtual ~PseudoInverse2() {
}

protected:
std::unique_ptr<Eigen::LDLT<Dynamic_Matrix> > _inverse_ptr;
Eigen::Matrix<Scalar, Eigen::Dynamic, 1> _damping;
Dynamic_Matrix _transposed, _inversed, _squared, _identity;
bool _type;

void _init(const Dynamic_Matrix& matrix)
{
        _inversed.setZero(matrix.cols(), matrix.rows());
        _transposed.setZero(matrix.cols(), matrix.rows());

        _type = (matrix.rows() > matrix.cols()) ? true : false;
        int size = (_type) ? matrix.rows() : matrix.cols();
        int other =  (_type) ? matrix.cols() : matrix.rows();
        _squared.setZero(other, other);
        _inverse_ptr.reset(new Eigen::LDLT<Dynamic_Matrix>(size));
        _damping = _damping.cwiseProduct(_damping);
        _identity.setIdentity(other, other);

//    std::cout << _type << std::endl;
//    std::cout << matrix.rows() << "\t" << matrix.cols() << std::endl;
}
};

//! Provides the computation of a step of a recursive algorithm for a null-space
// agumented projection of a matrix
/**
 * \param[in] J the matrix null-space projection is computed for
 * \param[in] P0 the null-space projection of a previous step
 * \param[in,out] P1 the new null-space projection matrix
 * \return an SVD pseudoinverse of a J*P matirx
 *
 */
template <typename _Matrix_Type_>
_Matrix_Type_ agumentedNullSpaceProjection(
        _Matrix_Type_ J, const _Matrix_Type_ P0, _Matrix_Type_& P1,
        double epsilon = std::numeric_limits<double>::epsilon())
{
        J = J * P0;
        _Matrix_Type_ p = pseudoInverse(J, epsilon);

        P1 = P0 - p * J;

        return p;
}

template <typename Dynamic_Matrix, typename Scalar> class AgumentedNullSpaceProjection
{

public:
AgumentedNullSpaceProjection(
        const Dynamic_Matrix& matrix,
        Scalar damping = std::numeric_limits<Scalar>::epsilon())
{
        _jacobian.setZero(matrix.rows(), matrix.cols());
        _inverser.init(matrix, damping);
//    _dets.setZero(matrix.rows());
}

AgumentedNullSpaceProjection(
        const Dynamic_Matrix& matrix,
        const Eigen::Matrix<Scalar, Eigen::Dynamic, 1>& damping)
{
        _jacobian.setZero(matrix.rows(), matrix.cols());
        _inverser.init(matrix, damping);
//    _dets.setZero(matrix.rows());
}

AgumentedNullSpaceProjection(
        const AgumentedNullSpaceProjection& other) : _jacobian(other._jacobian), _projector(other._projector), _inverser(other._inverser)
{
}

AgumentedNullSpaceProjection(
        AgumentedNullSpaceProjection&& other) : _jacobian(other._jacobian), _projector(other._projector), _inverser(other._inverser)
{
}

void compute(const Dynamic_Matrix& jacobian, Dynamic_Matrix& P)
{

        _jacobian.noalias() = jacobian * P;

        // here the singularity should be noticed?
        _inverser.compute(_jacobian);


        P.noalias() -= _inverser.get() * _jacobian;
}

Eigen::Matrix<Scalar, Eigen::Dynamic, 1> damping(){
  return _inverser.damping();
}

Scalar damping(int i){
        return _inverser.damping(i);
}

const Dynamic_Matrix& getInverse() const {
        return _inverser.get();
}
const Dynamic_Matrix& getJacobian() const {
        return _jacobian;
}

virtual ~AgumentedNullSpaceProjection() {
}



protected:
Dynamic_Matrix _jacobian, _projector;
PseudoInverse2<Dynamic_Matrix, Scalar> _inverser;
//  Dynamic_Matrix _dets;
};

Eigen::Matrix<bool, Eigen::Dynamic, 1>
flip(Eigen::Matrix<bool, Eigen::Dynamic, 1> vector);

inline void wrapToPi(double& th){
        th -= 6.28318531 * std::floor((th + 3.14159265) / 6.28318531);
}

//inline void limitToHalfPi(double& th){
//  th -= 3.14159265 * std::floor((th + 1.57079632679) / 3.14159265) + 1.57079632679;
//}


//template <typename _Matrix_Type_>
//inline void limitToHalfPi(_Matrix_Type_& vec){
//  for(int i = 0; i < std::max(vec.rows(), vec.cols()); i++) // How will it work for matrices?
//    limitToHalfPi(vec[i]);
//}

//template <typename _Matrix_Type_>
//inline _Matrix_Type_ limitToHalfPi(const _Matrix_Type_ vec){
//  _Matrix_Type_ limited = vec;
//  limitToHalfPi(vec, limited);
//  return limited;
//}

template <typename _Matrix_Type_>
inline void limitToHalfPi(const _Matrix_Type_& vec, _Matrix_Type_& limited){
//  _Matrix_Type_ limited = vec;
        for(int i = 0; i < std::max(vec.rows(), vec.cols()); i++) // How will it work for matrices?
                limitToHalfPi(vec[i], limited[i]);
}

template <typename _Matrix_Type_>
inline void limitToHalfPi(_Matrix_Type_& vec){
        limitToHalfPi(vec, vec);
}

template <> inline
void limitToHalfPi(const double& th, double& l_th){
        l_th = th - 3.14159265 * std::floor((th + 1.57079632) / 3.14159265);
}

template <typename _Vector_Type_, typename _Matrix_Type_>
void skew(const _Vector_Type_& vec, _Matrix_Type_& mat){

        mat << 0, -vec[2], vec[1], vec[2], 0, -vec[0], -vec[1], vec[0], 0;

}



template <typename _Matrix_Type1_, typename _Matrix_Type2_> inline
int limitHalfPi(_Matrix_Type1_ b_ref, _Matrix_Type2_& b){
        //  _Matrix_Type_ limited = vec;
        int factor = 0;

        for(int i = 0; i < std::max(b_ref.rows(), b_ref.cols()); i++) // How will it work for matrices?
                factor += limitHalfPi(b_ref[i], b[i]);

        return factor;
}

template <> inline
int limitHalfPi(double b_ref, double& b)
{
        int factor = (std::floor((b_ref + 1.57079632) / 3.14159265) -  std::floor((b + 1.57079632) / 3.14159265));
        b +=   3.14159265 * factor;
        return factor;
}

void limit2PI(double ref, double& st);

void limitPI(double ref, double& st);

// namespace std {

struct Hasher
{
        std::size_t operator()(const Eigen::Matrix<bool,Eigen::Dynamic,1>& k) const
        {
                using std::size_t;
                using std::hash;
                using std::string;

                std::vector<bool> std_vec(k.data(), k.data() + k.size());
                std::hash<std::vector<bool> >hash_bool;

                return hash_bool(std_vec);
        }
};

static Eigen::VectorXi iota(int size, int base = 0)
{
  Eigen::VectorXi iter;
  iter.setZero(size);

  for(int i = 0; i < size; i++)
    iter[i] = i + base;

  return iter;
}

static Eigen::VectorXi enumerate(Eigen::Matrix<bool, Eigen::Dynamic, 1> bools)
{
  Eigen::VectorXi ids(bools.count());
  int k = 0;
  for(int i = 0; i < bools.size(); i++){
    if(bools[i]) { ids[k] = i;
                   k++; }
  }

  return ids;
}

template <typename _Scalar, typename _Matrix> inline
std::vector<_Scalar> toVector(const _Matrix& m){
      return std::vector<_Scalar>(&m[0], m.data()+m.cols()*m.rows());
}



// }
// std::size_t hash(Eigen::Matrix<bool,Eigen::Dynamic,1> vec);

} // namespace package
} // namespace library
#endif
