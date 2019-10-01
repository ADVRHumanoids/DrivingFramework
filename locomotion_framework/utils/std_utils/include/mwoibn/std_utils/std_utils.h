#ifndef __MWOIBN_UTILS_STD_UTILS__STD_UTILS_H
#define __MWOIBN_UTILS_STD_UTILS__STD_UTILS_H

namespace mwoibn {
namespace std_utils {
inline int factorial(int n)
{
        return (n == 1 || n == 0) ? 1 : factorial(n - 1) * n;
}

class notImplemented : public std::logic_error
{
public:
    notImplemented(std::string function) : std::logic_error(function + ": Function not yet implemented") { };
};

class depracated : public std::logic_error
{
public:
    depracated(std::string function) : std::logic_error(function + ": Function is depracated due to an upredicted Eigen behaviour") { };
};

inline std::string separate(std::string full, std::string delimiter, std::string& back){

  std::string front;

  size_t pos = full.find(delimiter);
  if ( pos == std::string::npos) {
    front = "";
    back = full;
   }
  else{
    front = full.substr(0, pos);
    back = full.substr(pos+2);
  }

  return front;
}


}
}
#endif
