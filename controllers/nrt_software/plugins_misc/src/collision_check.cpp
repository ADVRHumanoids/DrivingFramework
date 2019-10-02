#include <mwoibn/visualization_tools/rviz_joint_limits.h>
#include <mwoibn/visualization_tools/rviz_distinc_links.h>

#include <mwoibn/robot_class/robot_ros_nrt.h>
#include <programs/string.h>

#include <ros/ros.h>
//#include <wrappers/ros_urdf_wrapper.h>

class Links
{

public:
Links() {
}
virtual ~Links() {
}
std::vector<std::string> getLinks() {
        return links;
}

bool updateLinks(programs::string::Request& req,
                 programs::string::Response& res)
{
        links.clear();
        std::cout << req.str.size() << std::endl;

        for (int i = 0; i < req.str.size(); i++)
        {
                links.push_back(req.str[i]);
                std::cout << req.str[i] << std::endl;
        }
        res.success = true;
}

private:
std::vector<std::string> links;
};

int main(int argc, char** argv)
{

        ros::init(argc, argv, "collision_check"); // initalize node

        ros::NodeHandle n;
        mwoibn::robot_class::RobotRosNRT robot("/robot_description", "", false, false,
                                               "joint_states", "/gazebo/link_states",
                                               false, true, 10);

        mwoibn::visualization_tools::RvizJointLimits joint_model(
                "rviz/model", robot, "/robot_description");

        mwoibn::visualization_tools::RvizDistincLinks highlight(
                "rviz/highlight", robot, "/robot_description");

        Links links;
        ros::Rate loop_rate(10);

        ros::ServiceServer service =
                n.advertiseService("update_links", &Links::updateLinks, &links);

        while (ros::ok())
        {

                joint_model.updateModel();
                joint_model.publish();

                highlight.updateLinks(links.getLinks());
                highlight.publish();
                ros::spinOnce();
                loop_rate.sleep();
        }

        return 0;
}
