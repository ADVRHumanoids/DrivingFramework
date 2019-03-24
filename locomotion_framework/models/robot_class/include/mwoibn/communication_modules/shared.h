#ifndef __MWOIBN__COMMUNICATION_MODULES__SHARED_H
#define __MWOIBN__COMMUNICATION_MODULES__SHARED_H

#include "mwoibn/common/all.h"

namespace mwoibn {

namespace communication_modules {

class Shared {

  typedef  mwoibn::VectorRT ShareObj;

public:
Shared() {}

Shared(const Shared& other): _shared(other._shared) { }

Shared( Shared&& other): _shared(other._shared) { }

virtual ~Shared() { }

bool add(const std::string& name, ShareObj share)
{
  _shared[name] = share;
  return true;
}


int size() const {
        return _shared.size();
}                                               // RT?

bool remove(const std::string& name)
{
  _shared.erase(name);
  return true;
}

bool has(const std::string& name){
  return _shared.count(name);
}

bool startsWith(const std::string& start){
  auto i = _shared.lower_bound(start);
  if (i != _shared.end())
         return (i->first.compare(0, start.size(), start) == 0); // Really a prefix?

  return false;
}

typename std::map<std::string, ShareObj>::iterator begin(){return _shared.begin();}
typename std::map<std::string, ShareObj>::iterator end(){return _shared.end();}

typename std::map<std::string, ShareObj>::const_iterator begin() const {return _shared.begin();}
typename std::map<std::string, ShareObj>::const_iterator end() const {return _shared.end();}

ShareObj& operator[](const std::string& name) {
        return _shared[name];
}

protected:
std::map<std::string, ShareObj> _shared;




///@}
};
} // namespace package
} // namespace library
#endif
