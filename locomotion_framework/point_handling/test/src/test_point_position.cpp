#include "mwoibn/point_handling/test.h"

#include "mwoibn/point_handling/point.h"


// Check if class initialization works correctly
TEST(PointTest, initialization)
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

  mwoibn::Vector3 P_1 = mwoibn::Vector3::Zero(3);
  P_1 << 0.0, 0.0, -0.15;

  try
  {
    // check initialization with one point by names
    mwoibn::point_handling::Point point_1(P_1, 8, *(_model_ptr));
  }
  catch (const std::exception& exc)
  {
    ADD_FAILURE() << exc.what();
  }
  catch (...)
  {
    ADD_FAILURE() << "Unknown exception";
  }

  try
  {
    // check initialization with one point by names
  mwoibn::point_handling::Point point_2(P_1, "arm2_7", *(_model_ptr));
  }
  catch (const std::exception& exc)
  {
    ADD_FAILURE() << exc.what();
  }
  catch (...)
  {
    ADD_FAILURE() << "Unknown exception";
  }

  EXPECT_THROW(mwoibn::point_handling::Point point_1(P_1, "asfd", *(_model_ptr)), std::invalid_argument);

}

TEST(PointTest, methods)
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

  mwoibn::Vector3 P_1 = mwoibn::Vector3::Zero(3);
  P_1 << 0.0, 0.0, -0.15;

  mwoibn::point_handling::Point point_1(P_1, "arm1_7", *(_model_ptr));

  mwoibn::point_handling::Point point_2("arm2_7", *(_model_ptr));

  mwoibn::VectorN joint_states(_model_ptr->dof_count);
  joint_states << 0.2, 0.2, -0.2, 0.5, -0.7, 0.6, -0.9, 0.5, 0.2, -0.2, 0.5,
      1.0, 0.6, 1.2, 0.5;

  RigidBodyDynamics::UpdateKinematicsCustom(*_model_ptr, &joint_states,
                                            NULL, NULL);
  point_1.getPositionReference("pelvis", joint_states);

//  std::cerr << point_2.getPositionFixed();
  EXPECT_EQ(mwoibn::Vector3::Zero(3), point_2.getPositionFixed());
}

