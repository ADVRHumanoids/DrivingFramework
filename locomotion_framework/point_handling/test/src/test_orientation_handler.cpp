#include "mwoibn/point_handling/test.h"
#include "mwoibn/point_handling/raw_orientations_handler.h"
#include "mwoibn/point_handling/point.h"

// Check if class initialization works correctly
TEST(OrientationPHTest, initialization)
{
  // initialize a robot
  RigidBodyDynamics::Model* _model_ptr = new RigidBodyDynamics::Model;

  try
  {
    std::string path = ros::package::getPath("point_handling");
    std::string file_name = path + "/test/resources/centauro.urdf";
    RigidBodyDynamics::Addons::URDFReadFromFile(file_name.c_str(), _model_ptr,
                                                false, false);

  }
  catch (...)
  {
    FAIL() << "Cannot load rbdl model abort";
  }

  mwoibn::Quaternion P_1 =
      mwoibn::Quaternion(0,0,0,1);


  EXPECT_NO_THROW(mwoibn::point_handling::RawOrientationsHandler point_1(
      _model_ptr->GetBodyId("pelvis"), *(_model_ptr)));
  EXPECT_NO_THROW(mwoibn::point_handling::RawOrientationsHandler point_1(
      "pelvis", *(_model_ptr)));

  mwoibn::point_handling::Point p1("arm1_7", *_model_ptr);
  mwoibn::point_handling::Point p2("arm1_7", *_model_ptr,
                                   "I have a name");
  EXPECT_NO_THROW(mwoibn::point_handling::RawOrientationsHandler point_1(
      _model_ptr->GetBodyId("pelvis"), *(_model_ptr),
      {p1, p2, p1}));
  EXPECT_NO_THROW(mwoibn::point_handling::RawOrientationsHandler point_1(
      "pelvis", *(_model_ptr), {p1, p2, p1}));

  unsigned int ref_id = _model_ptr->GetBodyId("pelvis");

  EXPECT_NO_THROW(mwoibn::point_handling::RawOrientationsHandler point_1(
      _model_ptr->GetBodyId("pelvis"), *(_model_ptr),
      {ref_id, ref_id, ref_id}));
  EXPECT_NO_THROW(mwoibn::point_handling::RawOrientationsHandler point_1(
      _model_ptr->GetBodyId("pelvis"), *(_model_ptr),
      {ref_id, ref_id, ref_id}, {P_1, P_1, P_1}));
  EXPECT_NO_THROW(mwoibn::point_handling::RawOrientationsHandler point_1(
      _model_ptr->GetBodyId("pelvis"), *(_model_ptr),
      {ref_id, ref_id, ref_id}, {}, {"a", "b", ""}));
  EXPECT_NO_THROW(mwoibn::point_handling::RawOrientationsHandler point_1(
      _model_ptr->GetBodyId("pelvis"), *(_model_ptr),
      {ref_id, ref_id, ref_id}, {P_1, P_1, P_1}, {"a", "b", ""}));

  EXPECT_NO_THROW(mwoibn::point_handling::RawOrientationsHandler point_1(
      "pelvis", *(_model_ptr), {"arm1_7", "arm1_7", "arm1_7"}));
  EXPECT_NO_THROW(mwoibn::point_handling::RawOrientationsHandler point_1(
      "pelvis", *(_model_ptr), {"arm1_7", "arm1_7", "arm1_7"},
      {P_1, P_1, P_1}));
  EXPECT_NO_THROW(mwoibn::point_handling::RawOrientationsHandler point_1(
      "pelvis", *(_model_ptr), {"arm1_7", "arm1_7", "arm1_7"}, {},
      {"a", "b", ""}));
  EXPECT_NO_THROW(mwoibn::point_handling::RawOrientationsHandler point_1(
      "pelvis", *(_model_ptr), {"arm1_7", "arm1_7", "arm1_7"},
      {P_1, P_1, P_1}, {"a", "b", ""}));

  EXPECT_THROW(
      mwoibn::point_handling::RawOrientationsHandler point_2(
          "asfd", *(_model_ptr), {"arm1_7", "arm2_7"}, {P_1, P_1}),
      std::invalid_argument);
  EXPECT_THROW(
      mwoibn::point_handling::RawOrientationsHandler point_2(
          "pelvis", *(_model_ptr), {"arm1_7", "fdgg"}, {P_1, P_1}),
      std::invalid_argument);
  EXPECT_THROW(
      mwoibn::point_handling::RawOrientationsHandler point_2(
          ref_id, *(_model_ptr), {"arm1_7", "fdgg"}, {P_1, P_1}),
      std::invalid_argument);
  EXPECT_THROW(mwoibn::point_handling::RawOrientationsHandler point_2(
                   "pelvis", *(_model_ptr), {"arm1_7", "arm1_7"},
                   {P_1, P_1, P_1}),
               std::invalid_argument);
  EXPECT_THROW(mwoibn::point_handling::RawOrientationsHandler point_2(
                   "pelvis", *(_model_ptr), {"arm1_7", "arm1_7"},
                   {P_1, P_1}, {"a", "b", ""}),
               std::invalid_argument);
  EXPECT_THROW(mwoibn::point_handling::RawOrientationsHandler point_2(
                   "pelvis", *(_model_ptr), {"arm1_7", "arm1_7"},
                   {P_1}, {"a", "b"}),
               std::invalid_argument);
  EXPECT_THROW(
      mwoibn::point_handling::RawOrientationsHandler point_2(
          "pelvis", *(_model_ptr), {"arm1_7", "arm1_7"}, {P_1}, {"a"}),
      std::invalid_argument);
  EXPECT_THROW(mwoibn::point_handling::RawOrientationsHandler point_2(
                   "pelvis", *(_model_ptr), {"arm1_7", "arm1_7"},
                   {P_1, P_1}, {"a"}),
               std::invalid_argument);

  mwoibn::point_handling::RawOrientationsHandler point_1("pelvis",
                                                    *(_model_ptr));
  mwoibn::point_handling::RawOrientationsHandler point_2(
      _model_ptr->GetBodyId("pelvis"), *(_model_ptr));

  EXPECT_EQ(ref_id, point_1.getOriginId());
  EXPECT_EQ(ref_id, point_2.getOriginId());
  EXPECT_EQ("pelvis", point_1.getOriginName());
  EXPECT_EQ("pelvis", point_2.getOriginName());
}

TEST(OrientationPHTest, JacobianAndStateMethods)
{

  RigidBodyDynamics::Model* _model_ptr = new RigidBodyDynamics::Model;

  try
  {
    std::string path = ros::package::getPath("point_handling");
    std::string file_name = path + "/test/resources/centauro.urdf";
    RigidBodyDynamics::Addons::URDFReadFromFile(file_name.c_str(), _model_ptr,
                                                false, false);

  }
  catch (...)
  {
    FAIL() << "Cannot load rbdl model abort";
  }


  mwoibn::VectorN joint_states(_model_ptr->dof_count);
  joint_states << 0.2, 0.2, -0.2, 0.5, -0.7, 0.6, -0.9, 0.5, 0.2, -0.2, 0.5,
      1.0, 0.6, 1.2, 0.5;

  RigidBodyDynamics::UpdateKinematicsCustom(*_model_ptr, &joint_states,
                                            NULL, NULL);

  // TEST FUNCTION THE DOES NOT REQUIRE EXTERNAL DATA
  mwoibn::Quaternion P_1 =
      mwoibn::Quaternion(0,0,0,1);

  // check initialization with two points by names
  mwoibn::point_handling::RawOrientationsHandler points(
      "torso_2", *_model_ptr, {"arm1_7", "arm2_7"}, {P_1, P_1});

  EXPECT_EQ(_model_ptr->dof_count, points.getFullJacobianCols());
  EXPECT_EQ(6, points.getFullJacobianRows());

  // TEST REST OF THE METHODS
  // try to open the file with correct values, if not abort as there is not
  // comparisson reference
  std::string path = ros::package::getPath("point_handling");
  std::string file_name = path + "/test/resources/point_handler.txt";
  std::ifstream myfile(file_name);
  if (!myfile.is_open())
    FAIL();

  // Prepare variables
  float eps = 0.0001;
  mwoibn::Matrix J_test;
  Eigen::MatrixXd J;

  // FIRST SET OF TESTS

  // FIRST MATRIX TEST
  // Skip to first line
  if (!mwoibn::point_handling::skipLine(&myfile, 2))
    FAIL() << "couldn't continue, file finished prematurely";
  // Read first matrix
  J = mwoibn::point_handling::readMatrix(&myfile, 3, 14);
  if (!mwoibn::point_handling::skipLine(&myfile, 3))
    FAIL() << "couldn't continue, file finished prematurely";
  J_test = points.getReducedPointJacobian(1, joint_states);
  // Test first matrix
  EXPECT_TRUE(mwoibn::point_handling::compareMatrices(J, J_test, eps));

//  std::cerr << "FIRST MATRIX" << std::endl;
//  std::cerr << "J" << std::endl;
//  std::cerr << J << std::endl;
//  std::cerr << "J_test" << std::endl;
//  std::cerr << J_test << std::endl;
  // SECOND MATRIX TEST
  if (!mwoibn::point_handling::skipLine(&myfile, 1))
    FAIL() << "couldn't continue, file finished prematurely";
  J = mwoibn::point_handling::readMatrix(&myfile, 3, _model_ptr->dof_count);
  if (!mwoibn::point_handling::skipLine(&myfile, 3))
    FAIL() << "couldn't continue, file finished prematurely";
  J_test = mwoibn::Matrix::Zero(points.getPointJacobianRows(1), _model_ptr->dof_count);
  points.getFullPointJacobian(1, J_test,
                              joint_states); // with orientation

//  std::cerr << "SECOND MATRIX" << std::endl;
//  std::cerr << "J" << std::endl;
//  std::cerr << J << std::endl;
//  std::cerr << "J_test" << std::endl;
//  std::cerr << J_test << std::endl;
  EXPECT_TRUE(mwoibn::point_handling::compareMatrices(J, J_test, eps));

  EXPECT_EQ(J.rows(), points.getPointJacobianRows(1));
  EXPECT_EQ(J.cols(), points.getFullPointJacobianCols(1));


  // TEST FULL JACOBIANS
  mwoibn::Matrix J_1 = mwoibn::Matrix::Zero(points.getPointJacobianRows(0), _model_ptr->dof_count);
  points.getFullPointJacobian(0, J_1,joint_states);
  mwoibn::Matrix J_2 = mwoibn::Matrix::Zero(points.getPointJacobianRows(1), _model_ptr->dof_count);
  points.getFullPointJacobian(1, J_2,joint_states);

  mwoibn::Matrix J_full = mwoibn::Matrix::Zero(points.getFullJacobianRows(), _model_ptr->dof_count);
  points.getFullJacobian(J_full, joint_states);

  EXPECT_TRUE(mwoibn::point_handling::compareMatrices(J_1, J_full.topRows(points.getPointJacobianRows(0)), eps));
  EXPECT_TRUE(mwoibn::point_handling::compareMatrices(J_2, J_full.bottomRows(points.getPointJacobianRows(1)), eps));

  std::vector<mwoibn::Matrix> v_J = points.getFullJacobians(joint_states);

  EXPECT_TRUE(mwoibn::point_handling::compareMatrices(J_1, v_J[0], eps));
  EXPECT_TRUE(mwoibn::point_handling::compareMatrices(J_2, v_J[1], eps));

  // TEST REDUCED JACOBIANS
  J_1 = points.getReducedPointJacobian(0, joint_states);
  J_2 = points.getReducedPointJacobian(1, joint_states);

  J_full = points.getReducedJacobian(joint_states);

  EXPECT_TRUE(mwoibn::point_handling::compareMatrices(J_1, J_full.topRows(points.getPointJacobianRows(0)), eps));
  EXPECT_TRUE(mwoibn::point_handling::compareMatrices(J_2, J_full.bottomRows(points.getPointJacobianRows(1)), eps));

  v_J = points.getReducedJacobians(joint_states);

  EXPECT_TRUE(mwoibn::point_handling::compareMatrices(J_1, v_J[0], eps));
  EXPECT_TRUE(mwoibn::point_handling::compareMatrices(J_2, v_J[1], eps));

}


