//#include "mgnss/nrt_software/plugins/joint_states.h"
#include "boost/program_options.hpp"
#include "mgnss/plugins/ros_shared.h"

//! Executeable to load ROS plugins
/** Command line arguments - name of the registered plugin to load - required
 *  Command line arguments - name of the plugin configuration - optional. If no plugin configuration is given a default equal to the registered plugin name
 */
int main(int argc, char** argv)
{


        // Read the command line arguments, name of the registered plugin and the configuration set up
        boost::program_options::options_description desc("Plugins");
        desc.add_options()
                ("plugin,p", boost::program_options::value<std::string>()->required(), "Plugin source");
        desc.add_options()
                ("name,n", boost::program_options::value<std::string>()->required(), "Plugin run-time name");

        boost::program_options::variables_map vm;

        try
        {
                boost::program_options::store( boost::program_options::parse_command_line(argc, argv, desc), vm);

                if ( !vm.count("plugin")  )
                {  std::cout << "Please specify plugin to load (p, plugin)" << std::endl;
                   return 1; }
        }
        catch(boost::program_options::error& e)
        {
                std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
                std::cerr << desc << std::endl;
                return 1;
        }
        catch(std::exception& e)
        {
                std::cerr << "Unhandled Exception reached the top of main: "
                          << e.what() << ", application will now exit" << std::endl;
                return 1;

        }

        // Load the requrested plugin
        std::unique_ptr<mgnss::plugins::RosBase> controller(mgnss::plugins::make(vm["plugin"].as<std::string>() ));

        // check if the different configuration name has been requested. If no, read the plugin configuration equal the registered plugin name
        std::string name = vm.count("name") ? vm["name"].as<std::string>(): vm["plugin"].as<std::string>();

        // connect the plugin with ROS
        controller->connect(argc, argv, name);
        // allocate the memmory
        controller->init();
        // wait for the robot feedbacks and set initial conditions
        controller->start(ros::Time::now().toSec());

        // run the control loop
        while(ros::ok()) {
                controller->control_loop(ros::Time::now().toSec()); // update the plugin
        }

        // shut down the plugin
        controller->close();
}
