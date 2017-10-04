#include <mwoibn/robot_class/robot_xbot_nrt.h>
#include <mwoibn/robot_class/robot_ros_nrt.h>
#include <mwoibn/gravity_compensation/simple_qr_gravity_compensation.h>
#include <mwoibn/motor_side_reference/sea_reference.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Empty.h>
#include <custom_messages/CustomCmnd.h>
#include <mwoibn/dynamic_models/qr_decomposition.h>

void switchMode(mwoibn::robot_class::Robot* robot_ptr,
                mwoibn::robot_class::Robot* robot_ref_ptr, bool* motor_side)
{

  if (*motor_side)
  {
    robot_ref_ptr->state.set(
        robot_ptr->command.get(mwoibn::robot_class::INTERFACE::POSITION),
        mwoibn::robot_class::INTERFACE::POSITION);
    robot_ref_ptr->update();
  }
  else
    robot_ptr->command.set(
        robot_ref_ptr->state.get(mwoibn::robot_class::INTERFACE::POSITION),
        mwoibn::robot_class::INTERFACE::POSITION);
}

void setReference(RigidBodyDynamics::Math::VectorNd* state_ptr,
                  mwoibn::robot_class::Robot* robot_ptr,
                  mwoibn::robot_class::Robot* robot_ref_ptr, bool* motor_side)
{
  if (*motor_side)
  {
    robot_ref_ptr->state.set(*state_ptr,
                             mwoibn::robot_class::INTERFACE::POSITION);
    robot_ref_ptr->update();
  }
  else
    robot_ptr->command.set(*state_ptr,
                           mwoibn::robot_class::INTERFACE::POSITION);
}
bool setMotorSideReference(std_srvs::SetBool::Request& req,
                           std_srvs::SetBool::Response& res,
                           mwoibn::robot_class::Robot* robot_ptr,
                           mwoibn::robot_class::Robot* robot_ref_ptr,
                           bool* motor_side)
{
  std::string enabled = "motor_side", disabled = "link_side";
  if (*motor_side == req.data)
  {
    res.success = true;
    res.message = "Reference has alredy been on " +
                  (*motor_side ? enabled : disabled) + "!";
    return true;
  }

  *motor_side = req.data;
  res.message =
      "Reference is now for " + (*motor_side ? enabled : disabled) + "!";
  switchMode(robot_ptr, robot_ref_ptr, motor_side);
  res.success = true;
  return true;
}

bool resetReference(std_srvs::Empty::Request& req,
                    std_srvs::Empty::Response& res,
                    mwoibn::robot_class::Robot* robot_ptr,
                    mwoibn::robot_class::Robot* robot_ref_ptr, bool* motor_side)
{
  RigidBodyDynamics::Math::VectorNd state =
      robot_ptr->state.get(mwoibn::robot_class::INTERFACE::POSITION);
  setReference(&state, robot_ptr, robot_ref_ptr, motor_side);
  return true;
}

void getContacts(const custom_messages::CustomCmnd::ConstPtr& msg,
                 mwoibn::robot_class::Robot* robot_ptr,
                 mwoibn::robot_class::Robot* robot_ref_ptr)
{

  for (int i = 0; i < robot_ptr->contacts().size(); i++)
  {
    //    std::cout << "for" << std::endl;

    if (msg->onlineGain1[i])
    {
      //      std::cout << "active" << std::endl;

      robot_ptr->contacts().contact(i).activate();
      robot_ref_ptr->contacts().contact(i).activate();
    }
    else
    {
      //      std::cout << "not active" << std::endl;
      robot_ptr->contacts().contact(i).deactivate();
      robot_ref_ptr->contacts().contact(i).deactivate();
    }
  }

  return;
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "centralized_controller"); // initalize node

  ros::NodeHandle n;
  bool motor_side = true;

  // init real robot
  mwoibn::robot_class::RobotRosNRT robot(
      "/home/user/malgorzata/workspace/src/locomotion_framework/configs/"
      "mwoibn_v2.yaml",
      "default", "/home/user/malgorzata/workspace/src/controllers/nrt_software/"
                 "configscentralized_controller.yaml");
  mwoibn::robot_class::RobotRosNRT robot_ref(
      "/home/user/malgorzata/workspace/src/locomotion_framework/configs/"
      "mwoibn_v2.yaml",
      "reference", "/home/user/malgorzata/workspace/src/controllers/"
                   "nrt_software/configscentralized_controller.yaml");

  // load controllers for offline GC
  //  mwoibn::gravity_compensation::SimpleQRGravityCompensation feed_forward(
  //      dynamic_model_ref, robot);
  //  mwoibn::motor_side_reference::SeaReference reference(robot, robot_ref,
  //  feed_forward);

  // load controllers for online GC
  mwoibn::dynamic_models::QrDecomposition dynamic_model(robot); // online set up
  mwoibn::gravity_compensation::SimpleQRGravityCompensation feed_forward(
      dynamic_model, robot);

  mwoibn::motor_side_reference::SeaReference reference(robot, feed_forward);

  ros::Subscriber sub = n.subscribe<custom_messages::CustomCmnd>(
      "CoM_regulator/contacts", 1,
      boost::bind(&getContacts, _1, &robot, &robot_ref));
  //  // debbuging purposes, switch between motor side and link side reference
  ros::ServiceServer switch_reference =
      n.advertiseService<std_srvs::SetBool::Request,
                         std_srvs::SetBool::Response>(
          "centralized_controller/motor_side_reference",
          boost::bind(&setMotorSideReference, _1, _2, &robot, &robot_ref,
                      &motor_side));

  // debbuging purposes, set current robot position as new reference
  ros::ServiceServer reset_reference =
      n.advertiseService<std_srvs::Empty::Request, std_srvs::Empty::Response>(
          "centralized_controller/reset_reference",
          boost::bind(&resetReference, _1, _2, &robot, &robot_ref,
                      &motor_side));
  robot.update();

  // set valid reference for current contact points
  robot.command.set(robot.state.get(mwoibn::robot_class::INTERFACE::POSITION),
                    mwoibn::robot_class::INTERFACE::POSITION);
  robot_ref.state.set(robot.state.get(mwoibn::robot_class::INTERFACE::POSITION),
                      mwoibn::robot_class::INTERFACE::POSITION);

  while (ros::ok())
  {
    feed_forward.update();

    robot.command.set(
        robot_ref.state.get(mwoibn::robot_class::INTERFACE::POSITION),
        mwoibn::robot_class::INTERFACE::POSITION);

    if (motor_side)
    {
      reference.update();
    }

   // robot_ref.update();
    robot.update();
  }
}
