#ifndef EIGEN_UTILS_EIGEN_UTILS_H
#define EIGEN_UTILS_EIGEN_UTILS_H

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
    _squared.setZero(size, size);
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
 * \param[in] P the null-space projection of a previous step
 * \param[in,out] the new null-space projection matrix
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

} // namespace package
} // namespace library
#endif
