//#include "mgnss/nrt_software/plugins/joint_states.h"
#include "boost/program_options.hpp"
#include "mgnss/plugins/ros_shared.h"

int main(int argc, char** argv)
{


        boost::program_options::options_description desc("Plugins");
        desc.add_options()
                ("plugin,p", boost::program_options::value<std::string>()->required(), "Plugin source");
        desc.add_options()
                ("name,n", boost::program_options::value<std::string>()->required(), "Plugin run-time name");

        boost::program_options::variables_map vm;

        try
        {
                boost::program_options::store( boost::program_options::parse_command_line(argc, argv, desc), vm); // can throw

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

        std::unique_ptr<mgnss::plugins::RosBase> controller(mgnss::plugins::make(vm["plugin"].as<std::string>() ));

        std::string name = vm.count("name") ? vm["name"].as<std::string>(): vm["plugin"].as<std::string>();

        controller->connect(argc, argv, name);
        controller->init();
        controller->start(ros::Time::now().toSec());

        while(ros::ok()) {
                controller->control_loop(ros::Time::now().toSec());
        }

        controller->close();
}
