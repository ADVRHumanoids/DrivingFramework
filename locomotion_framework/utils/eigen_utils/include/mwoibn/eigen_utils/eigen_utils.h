#ifndef EIGEN_UTILS_EIGEN_UTILS_H
#define EIGEN_UTILS_EIGEN_UTILS_H

#include <Eigen/Dense>
#include <Eigen/SVD>
#include <memory>
#include <iostream>
#include <cmath>

namespace mwoibn
{

namespace eigen_utils
{

const std::string PACKAGE = "eigen_utils";

//! Provides the SVD pseudoinverse for Eigen library
/**
 * \param[in] a the matrix pseudoinverse is computed for
 * \param[in] epsilon dafines numerical resultion of considered values, elements
 *of a matrix below this value are considered zero in copmutation of an inverse
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
                     svd.singularValues().array().abs()(0);
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

  PseudoInverse() {}

  void compute(const Dynamic_Matrix& matrix)
  {
    _svd_ptr->compute(matrix);

    _tolerance = _epsilon * _svd_ptr->singularValues().array().abs()(0);

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

  const Dynamic_Matrix& get() { return _inversedMatrix; }
  ~PseudoInverse() {}

protected:
  std::unique_ptr<Eigen::JacobiSVD<Dynamic_Matrix>> _svd_ptr;
  double _tolerance, _epsilon;
  Dynamic_Matrix _inversedMatrix, _tempMatrix, _dampedSingularValues;
};

template <typename Dynamic_Matrix> class PseudoInverse2
{

public:
  PseudoInverse2(const Dynamic_Matrix matrix,
                 double damping = std::numeric_limits<double>::epsilon())
  {
    init(matrix, damping);
    // compute(matrix);
  }

  PseudoInverse2() {}

  void compute(const Dynamic_Matrix& matrix)
  {
    _transposed.noalias() = matrix.transpose();
    if(_type)
     _squared.noalias() = _transposed * matrix;
    else
      _squared.noalias() = matrix * _transposed;

    for (int i = 0; i < _squared.rows(); i++)
      _squared(i, i) += _damping;

    _inverse_ptr->compute(_squared);
    _squared.noalias() = _inverse_ptr->solve(_identity);

    if (_type)
      _inversed.noalias() = _squared * _transposed;
    else
      _inversed.noalias() = _transposed * _squared;
  }

  void init(const Dynamic_Matrix matrix,
            double damping = std::numeric_limits<double>::epsilon())
  {
    _inversed.setZero(matrix.cols(), matrix.rows());
    _transposed.setZero(matrix.cols(), matrix.rows());

    _type = (matrix.rows() > matrix.cols()) ? true : false;
    int size = (_type) ? matrix.rows() : matrix.cols();
    int other =  (_type) ? matrix.cols() : matrix.rows();
    _squared.setZero(other, other);
    _inverse_ptr.reset(new Eigen::LDLT<Dynamic_Matrix>(size));
    _damping = damping * damping;
    _identity.setIdentity(other, other);

//    std::cout << _type << std::endl;
//    std::cout << matrix.rows() << "\t" << matrix.cols() << std::endl;
  }

  const Dynamic_Matrix& get() { return _inversed; }
  ~PseudoInverse2() {}

protected:
  std::unique_ptr<Eigen::LDLT<Dynamic_Matrix>> _inverse_ptr;
  double _damping;
  Dynamic_Matrix _transposed, _inversed, _squared, _identity;
  bool _type;
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

template <typename Dynamic_Matrix> class AgumentedNullSpaceProjection
{

public:
  AgumentedNullSpaceProjection(
      const Dynamic_Matrix matrix,
      double epsilon = std::numeric_limits<double>::epsilon())
  {
    _jacobian.setZero(matrix.rows(), matrix.cols());
    _inverser.init(matrix, epsilon);
  }

  void compute(const Dynamic_Matrix& jacobian, Dynamic_Matrix& P)
  {

    _jacobian.noalias() = jacobian * P;
    _inverser.compute(_jacobian);

    //std::cout << _jacobian << std::endl;

    P.noalias() -= _inverser.get() * _jacobian;
  }

  const Dynamic_Matrix& get() { return _inverser.get(); }

  ~AgumentedNullSpaceProjection() {}

protected:
  Dynamic_Matrix _jacobian;
  PseudoInverse2<Dynamic_Matrix> _inverser;
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

inline void limit2PI(double ref, double& st){
  if(st - ref > 1.57079632){
//    std::cout << "\t" << ref << "\t" << st << "\t";

    st -= 3.14159265;
//  std::cout << st << std::endl;
   limit2PI(ref, st);
  }
  else if (ref - st > 1.57079632){
//    std::cout << "\t" << ref << "\t" << st << "\t";
    st += 3.14159265;
//    std::cout << st << std::endl;
    limit2PI(ref, st);
  }
}

inline void limitPI(double ref, double& st){
  if(st - ref > 0.78539816){
//    std::cout << "\t" << ref << "\t" << st << "\t";

    st -= 1.57079632;
//  std::cout << st << std::endl;
   limitPI(ref, st);
  }
  else if (ref - st > 0.78539816){
//    std::cout << "\t" << ref << "\t" << st << "\t";
    st += 1.57079632;
//    std::cout << st << std::endl;
    limitPI(ref, st);
  }
}



} // namespace package
} // namespace library
#endif
