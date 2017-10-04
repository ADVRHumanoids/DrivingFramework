#include <gtest/gtest.h>
#include <ros/package.h>

#include "mwoibn/eigen_utils/eigen_utils.h"
#include <Eigen/Core>

#include <iostream>
#include <fstream>

bool skipLine(std::ifstream* myfile, int skip)
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

Eigen::MatrixXd readMatrix(std::ifstream* myfile, int rows, int cols)
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

  Eigen::Map<
      Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>
      J(&data[0], rows,
        cols); // requires c++11, "cannot-appear-in-a-constant-expression"

  return J;
}

::testing::AssertionResult compareMatrices(Eigen::MatrixXd m1,
                                           Eigen::MatrixXd m2, double eps)
{

  if (!m1.rows() == m2.rows())
    return ::testing::AssertionFailure()
           << " matrices has diffrenet row numbers"
           << "m1: " << m1.rows() << ", m2: " << m2.rows();
  if (!m1.cols() == m2.cols())
    return ::testing::AssertionFailure()
           << " matrices has diffrenet columns numbers"
           << "m1: " << m1.cols() << ", m2: " << m2.cols();

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

// Check if class initialization works correctly

// Check all class methods, orientation is not fully supported therefore it is
// not checked during tests
TEST(EigenUtilsTest, methods)
{

  // READ FILE
  std::string path = ros::package::getPath("eigen_utils");
  std::string file_name = path + "/test/resources/eigen_test.txt";
  std::ifstream myfile(file_name);
  if (!myfile.is_open())
    FAIL() << "couldn't open the file, abort";

  // Prepare variables
  float eps = 0.0001;
  Eigen::MatrixXd J_test;
  Eigen::MatrixXd J;

  // READ TEST MATRICES
  Eigen::MatrixXd m_square;
  Eigen::MatrixXd m_nonsquare;

  try
  {
    // read square matrix
    if (!skipLine(&myfile, 1))
      FAIL() << "couldn't continue, file finished prematurely";
    m_square = readMatrix(&myfile, 3, 3);

    // read non-sqaure
    if (!skipLine(&myfile, 1))
      FAIL() << "couldn't continue, file finished prematurely";
    m_nonsquare = readMatrix(&myfile, 5, 9);
  }
  catch (const std::exception& exc)
  {
    ADD_FAILURE() << exc.what();
  }
  catch (...)
  {
    ADD_FAILURE()
        << "Unknown exception, while getting PointHandling information";
  }

  // CHECK_METHODS
  try
  {
    // FIRST MATRIX TEST: pseudoInverse for square matrix with too low damping
    // Skip to first line
    if (!skipLine(&myfile, 1))
      FAIL() << "couldn't continue, file finished prematurely";
    // Read first matrix
    J = readMatrix(&myfile, 3, 3);
    // Compute test matrix
    J_test = mwoibn::eigen_utils::pseudoInverse(m_square, 1e-18);
    // Test first matrix
    EXPECT_TRUE(compareMatrices(J, J_test, 1));

    // SECOND MATRIX TEST: pseudoInverse for square matrix with correct damping
    if (!skipLine(&myfile, 1))
      FAIL() << "couldn't continue, file finished prematurely";
    J = readMatrix(&myfile, 3, 3);
    J_test = mwoibn::eigen_utils::pseudoInverse(m_square, 1e-8);
    EXPECT_TRUE(compareMatrices(J, J_test, eps));

    // THIRD MATRIX TEST: pseudoInverse for non-square matrix with defult
    // damping
    if (!skipLine(&myfile, 1))
      FAIL() << "couldn't continue, file finished prematurely";
    J = readMatrix(&myfile, 9, 5);
    J_test = Eigen::MatrixXd::Zero(9, 5);
    J_test = mwoibn::eigen_utils::pseudoInverse(m_nonsquare);
    EXPECT_TRUE(compareMatrices(J, J_test, eps));

    // FOURTH MATRIX TEST: agumented Null Space Projection Matrix
    Eigen::MatrixXd P;
    if (!skipLine(&myfile, 1))
      FAIL() << "couldn't continue, file finished prematurely";
    Eigen::MatrixXd P0 = readMatrix(&myfile, 9, 9);

    if (!skipLine(&myfile, 1))
      FAIL() << "couldn't continue, file finished prematurely";
    J = readMatrix(&myfile, 9, 5);
    J_test = Eigen::MatrixXd::Zero(9, 5);
    J_test = mwoibn::eigen_utils::agumentedNullSpaceProjection(m_nonsquare, P0, P);
    EXPECT_TRUE(compareMatrices(J, J_test, eps));
  }
  catch (const std::exception& exc)
  {
    ADD_FAILURE() << exc.what();
  }
  catch (...)
  {
    ADD_FAILURE()
        << "Unknown exception, while getting PointHandling information";
  }
}

int main(int argc, char** argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
