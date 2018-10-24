#include <vector>
#include <ros/ros.h>

#include <mwoibn/common/all.h>
#include <XBotCore-interfaces/XDomainCommunication.h>
#include <XBotInterface/Logger.hpp>
#include <custom_messages/CustomCmnd.h>

void poseCallback(const custom_messages::CustomCmnd::ConstPtr& msg,
                  XBot::PublisherNRT<mwoibn::VectorRT>* pub_com)
{
  static int base = 1;
  static int i = 0;
  mwoibn::VectorRT state;

  const int rt_size = mwoibn::rt_size-1;

  state.head(6) << msg->position[0], msg->position[1],
      msg->position[2], msg->position[3],
      msg->position[4], msg->position[5];

  state[rt_size] = mwoibn::IS_VALID;
  pub_com->write(state);

//  i++;
//  if (i == 20)
//  {
//    std::cout << "new\t" << i << "\t";
//    std::cout << state.head(6).transpose() << std::endl;
//    i = 0;
//  }

}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "floating_base_transform"); // initalize node

  ros::NodeHandle n;

  ros::Rate rate(200);

  XBot::PublisherNRT<mwoibn::VectorRT> pub_com("base_odometry_position");

  ros::Subscriber sub = n.subscribe<custom_messages::CustomCmnd>(
      "/centauro/base_odometry", 10, boost::bind(&poseCallback, _1, &pub_com));



  std::cout << "initialized floating_base publisher" << std::endl;

  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();
  }

  // let everyone know it unsubscribed
  mwoibn::VectorRT state;
  const int rt_size = mwoibn::rt_size-1;

  state.setZero();
  state[rt_size] = mwoibn::INVALID;

  pub_com.write(state);

  return 0;
}
