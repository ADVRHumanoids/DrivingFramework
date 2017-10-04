#include <ros/ros.h>
#include <gazebo_msgs/LinkStates.h>
#include <XBotCore-interfaces/XDomainCommunication.h>
#include <mwoibn/robot_class/robot_class.h>

void poseCallback(const gazebo_msgs::LinkStates::ConstPtr& msg,
                  XBot::PublisherNRT<mwoibn::VectorFS>* pub_com)
{
  static int base = 1;
  static int i = 0;
  mwoibn::VectorFS state;

  state << msg->pose[base].position.x, msg->pose[base].position.y,
      msg->pose[base].position.z, msg->pose[base].orientation.x,
      msg->pose[base].orientation.y, msg->pose[base].orientation.z,
      msg->pose[base].orientation.w, mwoibn::IS_VALID;
  pub_com->write(state);

  i++;
  if (i == 20)
  {
    std::cout << "new" << std::endl;
    std::cout << state.head(3) << std::endl;
    i = 0;
  }

}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "floating_base_transform"); // initalize node

  ros::NodeHandle n;

  ros::Rate rate(200);

  XBot::PublisherNRT<mwoibn::VectorFS> pub_com("floating_base");

  ros::Subscriber sub = n.subscribe<gazebo_msgs::LinkStates>(
      "/gazebo/link_states", 10, boost::bind(&poseCallback, _1, &pub_com));



  std::cout << "initialized floating_base publisher" << std::endl;

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  // let everyone know it unsubscribed
  mwoibn::VectorFS state;
  state << 0,0,0,0,0,0,0, mwoibn::INVALID;

  pub_com.write(state);

  return 0;
}
