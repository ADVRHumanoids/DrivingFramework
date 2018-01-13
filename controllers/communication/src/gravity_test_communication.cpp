#include <mwoibn/robot_class/robot.h>

#include <custom_services/updatePDGains.h>
#include <ros/ros.h>
#include <XBotCore-interfaces/XDomainCommunication.h>

bool wheelHandler(custom_services::updatePDGains::Request& req,
                  custom_services::updatePDGains::Response& res,
                  XBot::PublisherNRT<Eigen::Matrix<double, 4, 1>> *pub_com);

int main(int argc, char** argv)
{

  ros::init(argc, argv, "gravity_test"); // initalize node needed for the service

  ros::NodeHandle n;

  XBot::PublisherNRT<Eigen::Matrix<double, 4, 1>> pub_com;
  pub_com.init("com_position");

  ros::ServiceServer com_service =
      n.advertiseService<custom_services::updatePDGains::Request,
                         custom_services::updatePDGains::Response>(
          "trajectory", boost::bind(&wheelHandler, _1, _2, &pub_com));


  std::cout << "initialized com_position publisher" << std::endl;

  ros::Rate rate(10);

  while(ros::ok()){
    ros::spinOnce();
    rate.sleep();
  }
}

bool wheelHandler(custom_services::updatePDGains::Request& req,
                  custom_services::updatePDGains::Response& res,
                  XBot::PublisherNRT<Eigen::Matrix<double, 4, 1>> *pub_com )
{
  Eigen::Matrix<double, 4, 1> com_final;

  com_final[2] =  req.p/100.0;
  com_final[1] = req.d/100.0;
  com_final[0] = req.nr/100.0;
  com_final[3] = mwoibn::IS_VALID;

  pub_com->write(com_final);

  res.success = true;
  return true;
}
