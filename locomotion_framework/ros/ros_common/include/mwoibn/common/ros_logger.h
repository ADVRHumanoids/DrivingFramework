#ifndef __MWOIBN_COMMON_ROS_LOGGER_H
#define __MWOIBN_COMMON_ROS_LOGGER_H

#include "mwoibn/common/logger.h"
#include <iostream>
#include <fstream>
#include <iomanip>
#include <ctime>
#include <chrono>

namespace mwoibn {
namespace common {
class RosLogger : public Logger {

public:
RosLogger(std::string name){

        std::time_t t= std::time(nullptr);
        std::tm tm = *std::localtime(&t);

        std::ostringstream oss;

        oss << name << "_" << std::put_time(&tm, "%d-%m-%Y_%H-%M-%S") << ".txt";

        _file.open(oss.str(),  std::ios::out);

        if(_file.is_open())
                std::cout << "opened log _file " << oss.str() << std::endl;
        else
                std::cout << "couldn't open log _file " << oss.str() << std::endl;

        char cwd[1024];
        if(getcwd(cwd, sizeof(cwd)) != NULL)
                std::cout << "working directory\t" << cwd << std::endl;
        fmt.precision = 6;
        fmt.coeffSeparator = ", ";
        fmt.rowSeparator = ", ";
}
virtual ~RosLogger(){
}


virtual void write(){
        _file << _print.format(fmt) << "\n";
}

virtual void flush(){
        _file.flush();
}
virtual void close(){
        _file.close();
}



protected:
std::ofstream _file;
Eigen::IOFormat fmt;
std::map<std::string, int> _map;

//  double start,now;
mwoibn::VectorNT _print;

virtual void _start(){
        _file << "\n";
        flush();
}


virtual void _addField(const std::string& name, double init_value){
        if(!_print.size())
                _file << name;
        else
                _file << "," << name;
        _map.insert(std::pair<std::string, double>(name, _print.size()));
        _print.conservativeResize(_print.size()+1);
        _print.tail<1>()[0] = init_value;

}  //allocates memory, slow
virtual void _addEntry(const std::string& name, double value){
        _print[_map[name]] = value;
}  // doesn't allocate memory



};

}

}

#endif
