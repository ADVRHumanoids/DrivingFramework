#ifndef __MWOIBN_COMMON_LOGGER_H
#define __MWOIBN_COMMON_LOGGER_H

#include <string>

namespace mwoibn
{

namespace common {

class Logger {

public:
Logger(){
}
virtual ~Logger(){
}

virtual void addField(std::string name, double init_value) = 0;
virtual void addEntry(std::string name, double value) = 0;

virtual void write() = 0;

virtual void flush() = 0;
virtual void close() = 0;

virtual void start() = 0;
};

}

}
#endif
