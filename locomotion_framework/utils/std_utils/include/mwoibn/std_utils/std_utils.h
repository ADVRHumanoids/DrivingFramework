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


}
}
#endif
