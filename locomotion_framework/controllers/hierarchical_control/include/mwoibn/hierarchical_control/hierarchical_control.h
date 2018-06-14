#ifndef HIERARCHICAL_CONTROL_HIERARCHICAL_CONTROL_H
#define HIERARCHICAL_CONTROL_HIERARCHICAL_CONTROL_H

#include "mwoibn/simple_log/log.h"
#include "mwoibn/common/types.h"
#include <iostream>
namespace mwoibn {

namespace hierarchical_control {

enum class ACTIONS {
SNAP,
REPLACE,
};
const std::string PACKAGE  = "hierarchical_control";
} // namespace package
} // namespace library


#endif
