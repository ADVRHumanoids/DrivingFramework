#include "mwoibn/tests_common/test.h"
#include "mwoibn/point_handling/raw_positions_handler.h"
#include "mwoibn/point_handling/robot_points_handler.h"

#include "mwoibn/point_handling/position.h"

//typedef mwoibn::point_handling::RobotPointsHandler<mwoibn::point_handling::RawPositionsHandler>
//    PointsHandler;

typedef mwoibn::point_handling::PositionsHandler PointsHandler;

// Check if class initialization works correctly
TEST(RobotPHTest, initialization)
{
  // initialize a robot

  std::string path = ros::package::getPath("tests_common");
  std::string file_name = path + "/resources/urdf/centauro.urdf";

  mwoibn::robot_class::Robot robot(file_name, "",  true);

  mwoibn::Vector3 P_1 =
      mwoibn::Vector3::Zero(3);
  P_1 << 0.0, 0.0, -0.15;

  EXPECT_NO_THROW(PointsHandler point_1(robot.getModel().GetBodyId("pelvis"), robot));

  EXPECT_NO_THROW(PointsHandler point_1("pelvis", robot));

  mwoibn::point_handling::Position p1("arm1_7", robot.getModel());
  mwoibn::point_handling::Position p2("arm1_7", robot.getModel(), "I have a name");

  EXPECT_NO_THROW(PointsHandler point_1(
      robot.getModel().GetBodyId("pelvis"), robot, {p1, p2, p1}));

  EXPECT_NO_THROW(PointsHandler point_1("pelvis", robot, {p1, p2, p1}));

  unsigned int ref_id = robot.getModel().GetBodyId("pelvis");

  EXPECT_NO_THROW(
      PointsHandler point_1(robot.getModel().GetBodyId("pelvis"), robot,
                            {ref_id, ref_id, ref_id}));
  EXPECT_NO_THROW(
      PointsHandler point_1(robot.getModel().GetBodyId("pelvis"), robot,
                            {ref_id, ref_id, ref_id}, {P_1, P_1, P_1}));
  EXPECT_NO_THROW(
      PointsHandler point_1(robot.getModel().GetBodyId("pelvis"), robot,
                            {ref_id, ref_id, ref_id}, {}, {"a", "b", ""}));
  EXPECT_NO_THROW(PointsHandler point_1(
      robot.getModel().GetBodyId("pelvis"), robot,
      {ref_id, ref_id, ref_id}, {P_1, P_1, P_1}, {"a", "b", ""}));

  EXPECT_NO_THROW(
      PointsHandler point_1("pelvis", robot, {"arm1_7", "arm1_7", "arm1_7"}));
  EXPECT_NO_THROW(PointsHandler point_1(
      "pelvis", robot, {"arm1_7", "arm1_7", "arm1_7"}, {P_1, P_1, P_1}));
  EXPECT_NO_THROW(PointsHandler point_1(
      "pelvis", robot, {"arm1_7", "arm1_7", "arm1_7"}, {}, {"a", "b", ""}));
  EXPECT_NO_THROW(PointsHandler point_1("pelvis", robot,
                                        {"arm1_7", "arm1_7", "arm1_7"},
                                        {P_1, P_1, P_1}, {"a", "b", ""}));

  EXPECT_THROW(
      PointsHandler point_2("asfd", robot, {"arm1_7", "arm2_7"}, {P_1, P_1}),
      std::invalid_argument);
  EXPECT_THROW(
      PointsHandler point_2("pelvis", robot, {"arm1_7", "fdgg"}, {P_1, P_1}),
      std::invalid_argument);
  EXPECT_THROW(
      PointsHandler point_2(ref_id, robot, {"arm1_7", "fdgg"}, {P_1, P_1}),
      std::invalid_argument);
  EXPECT_THROW(PointsHandler point_2("pelvis", robot, {"arm1_7", "arm1_7"},
                                     {P_1, P_1, P_1}),
               std::invalid_argument);
  EXPECT_THROW(PointsHandler point_2("pelvis", robot, {"arm1_7", "arm1_7"},
                                     {P_1, P_1}, {"a", "b", ""}),
               std::invalid_argument);
  EXPECT_THROW(PointsHandler point_2("pelvis", robot, {"arm1_7", "arm1_7"},
                                     {P_1}, {"a", "b"}),
               std::invalid_argument);
  EXPECT_THROW(PointsHandler point_2("pelvis", robot, {"arm1_7", "arm1_7"},
                                     {P_1}, {"a"}),
               std::invalid_argument);
  EXPECT_THROW(PointsHandler point_2("pelvis", robot, {"arm1_7", "arm1_7"},
                                     {P_1, P_1}, {"a"}),
               std::invalid_argument);

  PointsHandler point_1("pelvis", robot);
  PointsHandler point_2(robot.getModel().GetBodyId("pelvis"), robot);

  EXPECT_EQ(ref_id, point_1.getOriginId());
  EXPECT_EQ(ref_id, point_2.getOriginId());
  EXPECT_EQ("pelvis", point_1.getOriginName());
  EXPECT_EQ("pelvis", point_2.getOriginName());
}
TEST(RobotPHTest, uniquePointMathods)
{ // check initialization with one point by names

  double eps = 0.0001;

  std::string path = ros::package::getPath("tests_common");
  std::string file_name = path + "/resources/urdf/centauro.urdf";

  mwoibn::robot_class::Robot robot(file_name, "", true );

  mwoibn::VectorN joint_states(robot.getDofs());
  joint_states << 0.2, 0.2, -0.2, 0.5, -0.7, 0.6, -0.9, 0.5, 0.2, -0.2, 0.5,
      1.0, 0.6, 1.2, 0.5;
  robot.state.set(joint_states, mwoibn::robot_class::INTERFACE::POSITION);
  robot.update();

  mwoibn::Vector3 P_1 =
      mwoibn::Vector3::Zero(3);
  P_1 << 0.0, 0.0, -0.15;
  mwoibn::Vector3 P_2 =
      mwoibn::Vector3::Zero(3);
  P_2 << 0.1, 0.0, -0.10;
  mwoibn::Vector3 P_3 =
      mwoibn::Vector3::Zero(3);
  P_3 << 0.2, 0.05, 0.10;
  mwoibn::Vector3 P_4 =
      mwoibn::Vector3::Zero(3);
  P_4 << 0.17, -0.05, 0.05;
  PointsHandler point_1("pelvis", robot, {"arm1_7", "arm2_7"}, {P_1, P_1});

  //  std::cerr << "-----------------------------\n     ADD/REMOVE "
  //               "POINTS\n-----------------------------" << std::endl;
  int i = 1;
  int max = 11;

  mwoibn::VectorN position_1 = point_1.getFullStateWorld();

  EXPECT_EQ(i + 1, point_1.addPoint(P_2, 8));
  i++;
  EXPECT_EQ(i + 1, point_1.addPoint(P_3, 8, "I have a name"));
  i++;
  EXPECT_EQ(i + 1, point_1.addPoint(P_4, "arm1_7"));
  i++;

  mwoibn::VectorN position_4 = point_1.getFullStateWorld();

  EXPECT_EQ(i + 1, point_1.addPoint(P_1, "arm1_7", "I have a name"));
  i++;
  EXPECT_EQ(i + 1, point_1.addPoint(8));
  i++;
  EXPECT_EQ(i + 1, point_1.addPoint(8, "I have a name"));
  i++;
  EXPECT_EQ(i + 1, point_1.addPoint("arm1_7"));
  i++;
  EXPECT_EQ(i + 1, point_1.addPoint("arm1_7", "I have a name"));
  i++;

  EXPECT_EQ(3, point_1.getPointId("I have a name"));
  i++;
  EXPECT_EQ(0, point_1.getPointId(""));
  i++;

  std::vector<unsigned int> ids = point_1.getPointIds("I have a name");
  EXPECT_EQ(4, ids.size());
  EXPECT_EQ(3, ids[0]);
  EXPECT_EQ(5, ids[1]);
  EXPECT_EQ(7, ids[2]);
  EXPECT_EQ(9, ids[3]);

  EXPECT_EQ("", point_1.getPointName(2));
  EXPECT_EQ("I have a name", point_1.getPointName(7));
  i++;

  mwoibn::point_handling::Position point("arm1_7", robot.getModel(), "test me");
  mwoibn::point_handling::Position point2(point);

  EXPECT_EQ(10, point_1.addPoint(point));
  i++;

  point_1.removePoint(10);
  point_1.removePoint(9);
  point_1.removePoint(8);
  point_1.removePoint(7);
  point_1.removePoint(6);
  point_1.removePoint(5);

  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(position_4, point_1.getFullStateWorld(), eps));

  point_1.removePoint(2);

  mwoibn::VectorN position_5(12);
  position_5.head(6) = position_1;
  position_5.tail(6) = position_4.tail(6);

  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(position_5, point_1.getFullStateWorld(), eps));

  point_1.removePoint(2);
  point_1.removePoint(2);

  std::string base = "pelvis";
  // Get data for get/set tests
  PointsHandler point_2(base, robot, {"arm1_7", "arm1_7"}, {P_1, P_1});

  mwoibn::point_handling::Position point_test_1(P_2, "arm1_7", robot.getModel(),
                                     "test me");
  mwoibn::point_handling::Position point_test_2(P_3, "arm1_7", robot.getModel(),
                                     "test me");

  mwoibn::Vector3 result_1_world =
      point_test_1.getStateWorld(robot.state.get(mwoibn::robot_class::INTERFACE::POSITION));
  mwoibn::Vector3 result_1_fixed =
      point_test_1.getStateFixed();
  mwoibn::Vector3 result_1_reference =
      point_test_1.getStateReference(base, robot.state.get(mwoibn::robot_class::INTERFACE::POSITION));
  mwoibn::Matrix jacobian_1 =
      point_test_1.getLinearStateJacobian(robot.state.get(mwoibn::robot_class::INTERFACE::POSITION));

  mwoibn::Vector3 result_2_world =
      point_test_2.getStateWorld(robot.state.get(mwoibn::robot_class::INTERFACE::POSITION));
  mwoibn::Vector3 result_2_fixed =
      point_test_2.getStateFixed();
  mwoibn::Vector3 result_2_reference =
      point_test_2.getStateReference(base, robot.state.get(mwoibn::robot_class::INTERFACE::POSITION));
  mwoibn::Matrix jacobian_2 =
      point_test_2.getLinearStateJacobian(robot.state.get(mwoibn::robot_class::INTERFACE::POSITION));

  // Make get/set tests

  point_2.setPointStateFixed(0, result_1_fixed);
  point_2.setPointStateFixed(1, result_2_fixed);

  EXPECT_TRUE(
      mwoibn::tests_common::compareMatrices(point_2.getPointStateWorld(0), result_1_world, eps));
  EXPECT_TRUE(
      mwoibn::tests_common::compareMatrices(point_2.getPointStateWorld(1), result_2_world, eps));

  point_2.setPointStateWorld(0, result_2_world);
  point_2.setPointStateWorld(1, result_1_world);

    EXPECT_TRUE(
        mwoibn::tests_common::compareMatrices(point_2.getPointStateReference(0),
                        result_2_reference, eps));
    EXPECT_TRUE(
        mwoibn::tests_common::compareMatrices(point_2.getPointStateReference(1),
                        result_1_reference, eps));

    point_2.setPointStateReference(0, result_1_reference);
    point_2.setPointStateReference(1, result_2_reference);

    EXPECT_TRUE(mwoibn::tests_common::compareMatrices(point_2.getPointStateFixed(0), result_1_fixed,
  eps));
    EXPECT_TRUE(mwoibn::tests_common::compareMatrices(point_2.getPointStateFixed(1), result_2_fixed,
  eps));

    EXPECT_TRUE(mwoibn::tests_common::compareMatrices(
        point_2.getPointJacobian(0), jacobian_1, eps));
    EXPECT_TRUE(mwoibn::tests_common::compareMatrices(
        point_2.getPointJacobian(1), jacobian_2, eps));

}

TEST(RobotPHTest, JacobianAndStateMethods)
{


  std::string path = ros::package::getPath("tests_common");
  std::string file_name = path + "/resources/urdf/centauro.urdf";

  mwoibn::robot_class::Robot robot(file_name, "", true);
  mwoibn::VectorN joint_states(robot.getDofs());
  joint_states << 0.2, 0.2, -0.2, 0.5, -0.7, 0.6, -0.9, 0.5, 0.2, -0.2, 0.5,
      1.0, 0.6, 1.2, 0.5;
  robot.state.set(joint_states, mwoibn::robot_class::INTERFACE::POSITION);
  robot.update();

  // TEST FUNCTION THE DOES NOT REQUIRE EXTERNAL DATA
  mwoibn::VectorN P_1(3);

  P_1 << 0.0, 0.0, -0.15;

  // check initialization with two points by names
  PointsHandler points(
      "torso_2", robot, {"arm1_7", "arm2_7"}, {P_1, P_1});

  EXPECT_EQ(robot.getDofs(), points.getFullJacobianCols());
  EXPECT_EQ(6, points.getFullJacobianRows());

  // TEST REST OF THE METHODS
  // try to open the file with correct values, if not abort as there is not
  // comparisson reference
  path = ros::package::getPath("point_handling");
  file_name = path + "/test/resources/point_handler.txt";
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
  if (!mwoibn::tests_common::skipLine(&myfile, 2))
    FAIL() << "couldn't continue, file finished prematurely";
  // Read first matrix
  if (!mwoibn::tests_common::skipLine(&myfile, 3))
    FAIL() << "couldn't continue, file finished prematurely";
  J = mwoibn::tests_common::readMatrix(&myfile, 3, 14);
  J_test = points.getReducedPointJacobian(1);
  // Test first matrix
  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(J, J_test, eps));

  // SECOND MATRIX TEST
  if (!mwoibn::tests_common::skipLine(&myfile, 1))
    FAIL() << "couldn't continue, file finished prematurely";
  if (!mwoibn::tests_common::skipLine(&myfile, 3))
    FAIL() << "couldn't continue, file finished prematurely";
  J = mwoibn::tests_common::readMatrix(&myfile, 3, robot.getDofs());
  J_test = mwoibn::Matrix::Zero(3, robot.getDofs());
  points.getFullPointJacobian(1, J_test); // with orientation

  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(J, J_test, eps));

  EXPECT_EQ(J.rows(), points.getPointJacobianRows(1));
  EXPECT_EQ(J.cols(), points.getFullPointJacobianCols(1));

  // THIRD MATRIX TEST
  if (!mwoibn::tests_common::skipLine(&myfile, 1))
    FAIL() << "couldn't continue, file finished prematurely";
  J = mwoibn::tests_common::readMatrix(&myfile, 6, 14);
  J_test = points.getReducedJacobian();
  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(J, J_test, eps));

  std::vector<mwoibn::Matrix> jacobians =
points.getReducedJacobians();

  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(J.topRows(3), jacobians[0], eps));
  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(J.bottomRows(3), jacobians[1], eps));


  // FOURTH MATRIX TEST
  if (!mwoibn::tests_common::skipLine(&myfile, 1))
    FAIL() << "couldn't continue, file finished prematurely";
  J = mwoibn::tests_common::readMatrix(&myfile, 6, 15);
  J_test = mwoibn::Matrix::Zero(6, robot.getDofs());
  points.getFullJacobian(J_test); // only position

  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(J, J_test, eps));

  jacobians.clear();

  jacobians = points.getFullJacobians();
  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(J.topRows(3), jacobians[0], eps));
  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(J.bottomRows(3), jacobians[1], eps));

  EXPECT_EQ(J.rows(), points.getFullJacobianRows());
  EXPECT_EQ(J.cols(), points.getFullJacobianCols());


  // FIFTH MATRIX TEST
  if (!mwoibn::tests_common::skipLine(&myfile, 1))
    FAIL() << "couldn't continue, file finished prematurely";
  if (!mwoibn::tests_common::skipLine(&myfile, 3))
    FAIL() << "couldn't continue, file finished prematurely";

  // SIXTH MATRIX TEST
  if (!mwoibn::tests_common::skipLine(&myfile, 1))
    FAIL() << "couldn't continue, file finished prematurely";
  if (!mwoibn::tests_common::skipLine(&myfile, 7))
    FAIL() << "couldn't continue, file finished prematurely";


  // SEVENTH MATRIX TEST
  if (!mwoibn::tests_common::skipLine(&myfile, 1))
    FAIL() << "couldn't continue, file finished prematurely";
  J = mwoibn::tests_common::readMatrix(&myfile, 6, 1);
  J_test = mwoibn::Matrix::Zero(6, 1);
  J_test = points.getFullStateWorld();

  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(J_test, J, eps));

  std::vector<mwoibn::Vector3> states =
points.getFullStatesWorld();

  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(J.topRows(3), states[0], eps));
  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(J.bottomRows(3), states[1], eps));

  mwoibn::point_handling::Position point1(P_1, "arm1_7", robot.getModel());
  mwoibn::point_handling::Position point2(P_1, "arm2_7", robot.getModel());

  mwoibn::VectorN ref_position(6);

  ref_position.head(3) = point1.getStateReference("torso_2",
robot.state.get(mwoibn::robot_class::INTERFACE::POSITION));
  ref_position.tail(3) = point2.getStateReference("torso_2",
robot.state.get(mwoibn::robot_class::INTERFACE::POSITION));

  J_test = points.getFullStateReference();

  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(J_test, ref_position, eps));

  states.clear();
  states = points.getFullStatesReference();

  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(J_test.topRows(3), states[0], eps));
  EXPECT_TRUE(mwoibn::tests_common::compareMatrices(J_test.bottomRows(3), states[1], eps));

}

