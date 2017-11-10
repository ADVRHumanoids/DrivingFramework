#ifndef ROBOT_CLASS_MAPPINGS_H
#define ROBOT_CLASS_MAPPINGS_H

#include "mwoibn/robot_class/map.h"

namespace mwoibn
{
namespace robot_class
{

// Handles multiple maps, each map has to have a unique name
template <typename Map>
class Mappings{

public:
  Mappings(){}
  ~Mappings(){}

  bool addMap(Map map){

    // add checking if the map has already been defined
    if(isDefined(map.getName())) return false;

    _mappings.push_back(map);
    return true;
  }

  const Map& get(int i) const {
    return _mappings[i];
  }

  /** return a reference to a specific map.
   *
   * It does not check whether the mapping exists
   *
   * @see isDefined
   */
  const Map& get(std::string name) const {
      return _mappings[getId(name)];
  }

  bool isDefined(std::string name) const{
      auto it = std::find_if(_mappings.begin(), _mappings.end(),
                             [name](const Map& point)
                             {
                               return point.getName() == name;
                             });
      if(it == _mappings.end()) return false;

      return true;
  }

  int size(){return _mappings.size();}

  /** @brief returns ID of a given mapping, retruns robot_class::NON_EXISTING it map is not defined
   */
  int getId(std::string name) const {
      auto it = std::find_if(_mappings.begin(), _mappings.end(),
                             [name](const Map& point)
                             {
                               return point.getName() == name;
                             });
      if(it == _mappings.end()) return robot_class::NON_EXISTING;

      return it - _mappings.begin();
  }

protected:
  std::vector<Map> _mappings;

};

} // namespace package
} // namespace library


#endif // MAPPINGS_H
