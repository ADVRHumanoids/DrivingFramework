#ifndef __MWOIBN_COMMON_LOGGER_H
#define __MWOIBN_COMMON_LOGGER_H

#include <string>
#include <functional>

namespace mwoibn
{

namespace common {

class Logger {

public:
Logger(){
  _full_name.reserve(1000);
}
virtual ~Logger(){
}

virtual void write() = 0;

virtual void flush() = 0;
virtual void close() = 0;

void start(){
  _start();
  add = std::bind(&Logger::_addEntry, this, std::placeholders::_1, std::placeholders::_2);
}
//std::function<void(Foo*)> f = &Foo::doSomething;
std::function<void(const std::string&, double)> add = std::bind(&Logger::_addField, this, std::placeholders::_1, std::placeholders::_2);
void prefix(const std::string& prefix){_prefix = prefix;}

protected:
  std::string _prefix = "";
  std::string _full_name = "";
  virtual void _addField(const std::string& name, double init_value) = 0;
  virtual void _addEntry(const std::string& name, double value) = 0;
  virtual void _start() = 0;
};

}

}
#endif
