#include <mwoibn/robot_class/robot_ros_nrt.h>
#include <mwoibn/robot_class/robot_xbot_nrt.h>
#include <mwoibn/robot_class/robot_xbot_rt.h>
#include <config.h>

#include <mwoibn/hierarchical_control/hierarchical_controller.h>
#include <mwoibn/hierarchical_control/joint_positions_task.h>
#include <mwoibn/hierarchical_control/center_of_mass_task.h>
#include <mwoibn/hierarchical_control/constraints_task.h>
#include <mwoibn/hierarchical_control/controller_task.h>
#include <mwoibn/hierarchical_control/cartesian_selective_task.h>
#include <mwoibn/hierarchical_control/orientation_world_task.h>
#include <mwoibn/hierarchical_control/orientation_selective_task.h>

#include <mwoibn/point_handling/robot_points_handler.h>
#include <custom_services/updatePDGains.h>
#include <custom_services/updatePrint.h>


#include <custom_services/updatePDGains.h>
#include <mwoibn/eigen_utils/eigen_utils.h>

bool referenceHandler(custom_services::updatePDGains::Request& req,
                  custom_services::updatePDGains::Response& res,
                  double* step, double *vel, double *x);

int main(int argc, char** argv)
{

  ros::init(argc, argv,
            "gravity_test"); // initalize node needed for the service

  ros::NodeHandle n;
  ros::Rate rate(200);

  std::string path = std::string(DRIVING_FRAMEWORK_WORKSPACE);
//  mwoibn::robot_class::RobotRosNRT robot(path+"DrivingFramework/locomotion_framework/configs/mwoibn_v2.yaml", "higher_scheme", path+"controllers/nrt_software/configs/centralized_controller.yaml");
   mwoibn::robot_class::RobotXBotNRT robot(path+"DrivingFramework/locomotion_framework/configs/mwoibn_v2.yaml", "joint_space");
  //  robot.contacts().contact(1).deactivate();

  double pos = 0, vel = 0, step = 0.0001;
  ros::ServiceServer trajectory_service =
      n.advertiseService<custom_services::updatePDGains::Request,
                         custom_services::updatePDGains::Response>(
          "trajectory", boost::bind(&referenceHandler, _1, _2, &pos, &vel, &step));

    std::cout << "Dofs: " << robot.getDofs() << std::endl;

  mwoibn::VectorN command_0(robot.getDofs()), command_1(robot.getDofs()), command_2(robot.getDofs()), velocities_0(robot.getDofs()), velocities_1(robot.getDofs()), velocities_2(robot.getDofs());
  mwoibn::VectorN command(robot.getDofs()), velocities(robot.getDofs());
  mwoibn::VectorN des_pos(robot.getDofs()), des_vel(robot.getDofs());

//  command << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//      3.14 / 2, 3.14 / 4, 3.14 / 4, 0, 0, 0,
//      -3.14 / 2, -3.14 / 4, -3.14 / 4, 0, 0, 0,
//      -3.14 / 2, -3.14 / 4, -3.14 / 4, 0, 0, 0,
//      3.14 / 2, 3.14 / 4, 3.14 / 4, 0, 0, 0,
//      0, 0.0, -0.5236, -0.5236, -0.7854, 0.0, -0.7854, 0.0,
//      0.0, 0.0, 0.5236, 0.5236, 0.7854,  0.0, 0.7854,  0.0,
//      0.0;

//  command << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 3.14 / 2, 3.14 / 4, 3.14 / 4, 0, 0,
//      0, -3.14 / 2, -3.14 / 4, -3.14 / 4, 0, 0, 0, -3.14 / 2, -3.14 / 4,
//      -3.14 / 4, 0, 0, 0, 3.14 / 2, 3.14 / 4, 3.14 / 4, 0, 0, 0, 0, 0.0,
//      -0.5236, -0.5236, -0.7854, 0.0, -0.7854, 0.0, 0.0, 0.5236, 0.5236, 0.7854,
//      0.0, 0.7854, 0.0;


//  command << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//      0, -1.57, -2.41, -0.90, 0, 0,
//      0,  1.57,  2.41,  0.90, 0, 0,
//      0,  1.57,  2.41,  0.90, 0, 0,
//      0, -1.57, -2.41, -0.90, 0, 0,
//      0.0, 0.0, -0.5236, -0.5236, -0.7854, 0.0, -0.7854,
//      0.0, 0.0,  0.5236,  0.5236,  0.7854, 0.0, 0.7854, 0.0;

//    command << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//        3.14 / 2, 3.14 / 4, 3.14 / 4, 0, 0, 0,
//        -3.14 / 2, -3.14 / 4, -3.14 / 4, 0, 0, 0,
//        -3.14 / 2, -3.14 / 4, -3.14 / 4, 0, 0, 0,
//        3.14 / 2, 3.14 / 4, 3.14 / 4, 0, 0, 0,


  command_0 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
       1.5,  0.8,  0.8, 0.0,  1.5, 0.0,
      -1.5, -0.8, -0.8, 0.0, -1.5, 0.0,
      -1.5, -0.8, -0.8, 0.0, -1.5, 0.0,
       1.5,  0.8,  0.8, 0.0,  1.5, 0.0;

  command_1 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0,  -0.0, -0.0,  0.0, 0.0, 0.0,
      0.0,   0.0,  0.0, -0.0, 0.0, 0.0,
      0.0,   0.0,  0.0, -0.0, 0.0, 0.0,
      0.0,  -0.0, -0.0,  0.0, 0.0, 0.0;


  command_2 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0, -1.57, -1.57, -0.0, 0, -0.0,
      0,  1.57,  1.57,  0.0, 0, 0.0,
      0,  1.57,  1.57,  0.0, 0, 0.0,
      0, -1.57, -1.57, -0.0, 0, -0.0;

//  command << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//      0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

  velocities << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0, 0.0, 0.0, -0.0, 0, -0.0,
      0, 0.0, 0.0,  0.0, 0,  0.0,
      0, 0.0, 0.0,  0.0, 0, -0.0,
      0, 0.0, 0.0, -0.0, 0,  0.0;
      
  velocities_0 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0, 0.0, 0.0, -0.0, 0, -0.0,
      0, 0.0, 0.0,  0.0, 0,  0.0,
      0, 0.0, 0.0,  0.0, 0, -0.0,
      0, 0.0, 0.0, -0.0, 0,  0.0;

  velocities_1 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0, 0.0, 0.0, -0.0, 0, -0.5,
      0, 0.0, 0.0,  0.0, 0,  0.5,
      0, 0.0, 0.0,  0.0, 0, -0.5,
      0, 0.0, 0.0, -0.0, 0,  0.5;

  velocities_2 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0, 0.0, 0.0, -0.0, 0, -1.0,
      0, 0.0, 0.0,  0.0, 0,  1.0,
      0, 0.0, 0.0,  0.0, 0, -1.0,
      0, 0.0, 0.0, -0.0, 0,  1.0;

  command = robot.state.get(mwoibn::robot_class::INTERFACE::POSITION);
  des_pos = robot.state.get(mwoibn::robot_class::INTERFACE::POSITION);
  des_vel = velocities;
      
  
//    command << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//        0, -1.57, -2.41, -0.90, 0, 0,
//        0,  1.57,  2.41,  0.90, 0, 0,
//        0,  1.57,  2.41,  0.90, 0, 0,
//        0, -1.57, -2.41, -0.90, 0, 0,

//  command << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
//            0, -1.57, -2.41, -0.90, 0, 0,
//            0,  1.57,  2.41,  0.90, 0, 0,
//            0,  1.57,  2.41,  0.90, 0, 0,
//            0, -1.57, -2.41, -0.90, 0, 0,
//      0, 0.0, -0.5236, -0.5236, -0.7854, 0.0, -0.7854, 0.0,
//      0.0, 0.0, 0.5236, 0.5236, 0.7854,  0.0, 0.7854,  0.0,
//      0.0;

      
  while (ros::ok())
  {
    if(pos == 1)
      des_pos = command_0;
    else if(pos == 2)
      des_pos = command_1;
    else if(pos == 3)
      des_pos = command_2;
    else if(pos == 4)
      des_pos = robot.state.get(mwoibn::robot_class::INTERFACE::POSITION);
    
    if(vel == 0)
        des_vel = velocities_0;
    else if(vel == 1)
        des_vel = velocities_1;
    else if(vel == 2)
        des_vel = velocities_2;

    
    for(int i = 0; i < command.size(); i++){
     if (std::fabs(des_pos[i] - command[i]) > step){
      if (des_pos[i] - command[i] > 0)
        command[i] += step;
      else
        command[i] -= step;
    }
    else
        command[i] = des_pos[i];
    
    if (std::fabs(des_vel[i] - des_vel[i]) > 0.1){
      if (des_vel[i] - velocities[i] > 0)
        velocities[i] += 0.1;
      else
        velocities[i] -= 0.1;
    }
    else
        velocities[i] = des_vel[i];

    }
        
    robot.command.set(command, mwoibn::robot_class::INTERFACE::POSITION);
    robot.command.set(velocities, mwoibn::robot_class::INTERFACE::VELOCITY);

    robot.update();
  }
}

bool referenceHandler(custom_services::updatePDGains::Request& req,
                  custom_services::updatePDGains::Response& res,
                  double* pos, double* vel, double* step)
{

  *pos =  req.p;
  *vel =  req.d;
  *step = req.nr/10000.0;
  res.success = true;
  return true;
}
