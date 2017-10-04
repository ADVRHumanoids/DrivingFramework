#ifndef TESTS_COMMON_TEST_H
#define TESTS_COMMON_TEST_H

#include <gtest/gtest.h>
#include <ros/package.h>

#include <rbdl/rbdl.h>
#include <rbdl/addons/urdfreader/urdfreader.h>
#include "mwoibn/robot_class/robot.h"

#include <iostream>
#include <fstream>

namespace mwoibn {

namespace tests_common {

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

template <typename Matrix1, typename Matrix2>
inline ::testing::AssertionResult compareMatrices(Matrix1 m1,
                                           Matrix2 m2, double eps)
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

template <typename Vector>
inline ::testing::AssertionResult compareStdVectors(Vector m1,
                                           Vector m2, double eps)
{

  if (m1.size() != m2.size())
    return ::testing::AssertionFailure()
           << " vectors have diffrenet sizes";

  for (int i = 0; i < m1.size(); i++)
  {
      if (fabs(m1[i] - m2[i]) > eps)
        return ::testing::AssertionFailure() << "row " << i
                                             << ", value a " << m1[i]
                                             << ", value b " << m2[i];
  }

  return ::testing::AssertionSuccess();
}

inline std::unique_ptr<mwoibn::robot_class::Robot> _initTestRobot(bool is_static)
{
std::unique_ptr<mwoibn::robot_class::Robot> robot_ptr;
std::string path;
std::string file_name;
// Try to load a model, if fail abort, as collision model depends on
// mwoibn::robot_class
path = ros::package::getPath("tests_common");
file_name = path + "/resources/urdf/centauro.urdf";

robot_ptr.reset(new mwoibn::robot_class::Robot(file_name, "", true));

return std::move(robot_ptr);
}

} // namespace package
} // namespace library

#endif
