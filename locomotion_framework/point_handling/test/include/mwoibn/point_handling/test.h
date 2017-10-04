#ifndef POINT_HANDLING_TEST_H
#define POINT_HANDLING_TEST_H

#include <gtest/gtest.h>
#include <ros/package.h>

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>

#include <iostream>
#include <fstream>

namespace mwoibn {
namespace point_handling {

inline bool skipLine(std::ifstream* myfile, int skip)
{
  std::string line;
  try
  {
    for (int i = 0; i < skip; i++)
      getline(*myfile, line);
  }
  catch (const std::exception& exc)
  {
    ADD_FAILURE() << exc.what(); // add proper throws
  }
  return true;
}

inline Eigen::MatrixXd readMatrix(std::ifstream* myfile, int rows, int cols)
{

  std::string line;

  std::vector<double> data;
  std::vector<double> new_data;
  for (int i = 0; i < rows; i++)
  {
    getline(*myfile, line);
    std::istringstream is(line);
    new_data = std::vector<double>(std::istream_iterator<double>(is),
                                   std::istream_iterator<double>());
    data.insert(data.end(), new_data.begin(), new_data.end());
  }

  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic,
                           Eigen::RowMajor>> J(&data[0], rows, cols); // requires c++11, "cannot-appear-in-a-constant-expression"
  return J;
}

inline ::testing::AssertionResult compareMatrices(Eigen::MatrixXd m1,
                                           Eigen::MatrixXd m2, double eps)
{

  if (!m1.rows() == m2.rows())
    return ::testing::AssertionFailure()
           << " matrices has diffrenet row numbers";
  if (!m1.cols() == m2.cols())
    return ::testing::AssertionFailure()
           << " matrices has diffrenet columns numbers";

  for (int i = 0; i < m1.rows(); i++)
  {
    for (int j = 0; j < m1.cols(); j++)
    {
      if (fabs(m1(i, j) - m2(i, j)) > eps)
        return ::testing::AssertionFailure() << "row " << i << ", columns " << j
                                             << ", value a " << m1(i, j)
                                             << ", value b " << m2(i, j);
    }
  }

  return ::testing::AssertionSuccess();
}

} // namespace package
} // namespace library
#endif
