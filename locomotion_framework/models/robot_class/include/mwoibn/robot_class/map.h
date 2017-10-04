#ifndef ROBOT_CLASS_MAP_H
#define ROBOT_CLASS_MAP_H

#include "mwoibn/robot_class/robot_class.h"

namespace mwoibn
{
namespace robot_class
{

/** @brief basic class holding mapping between different systems
 *
 * It works with integers and supports negative and repeating entries, therefore a reversed
 * mapping is not supported.
 *
 * Entries for which the mapping is not defined are marked as NON_EXISTING
 * defined in robot_class.
 *
 */
class Map
{
public:
  Map(std::string name, VectorInt map) : _name(name), _map(map) {}

  //    int max = 0;

  //    for(int i = 0; i < _map.size(); i++){
  //      if(_map[i] > max && _map[i] != robot_class::NON_EXISTING)

  //    }
  //    _reversed.setConstant(max, robot_class::NON_EXISTING);

  virtual ~Map() {}

  std::string getName() const { return _name; }
  VectorInt get() const { return _map; }
  double getDofs() const { return _map.size(); }

protected:
  std::string _name;
  VectorInt _map;
};

/** @brief Extenstion of Map class to a bidirectional mapping 
 *
 * It works with integers due to the Map inheritence. On intialization it checks
 * the vector for negative and repeated entries and throws an error in case it finds one.
 *
 * An additional reverse mapping is defined for these maps.
 *
 * Entries for which the mapping is not defined are marked as NON_EXISTING
 * defined in robot_class.h.
 *
 */
class BiMap : public Map
{
public:
  BiMap(std::string name, VectorInt map) : Map(name, map)
  {
    int max = 0;

    //check for negative values in a vector
    if(_map.minCoeff() < 0)
      throw(std::invalid_argument("The negative values are not supported in bidirectional maps"));

    for (int i = 0; i < _map.size(); i++)
    {
      if (_map[i] > max && _map[i] != robot_class::NON_EXISTING)
        max = map[i];
    }

    _reversed.setConstant(max+1, robot_class::NON_EXISTING);

    for(int i = 0; i < _map.size(); i++){
      if(_map[i] == robot_class::NON_EXISTING) continue;
      // check for repeating entries
      if(_reversed[_map[i]] != robot_class::NON_EXISTING){
        std::stringstream err;
        err << "The repeating values are not supported in bidirectional maps found " <<_map[i] << " twice.";
        throw(std::invalid_argument(err.str().c_str()));
      }
      _reversed[_map[i]] = i;
    }
  }

  virtual ~BiMap() {}

  const VectorInt& reversed() const { return _reversed; }

  /** @brief Maps vectors using defined map
   *
   * if map is defined as q[a] = b
   *
   * it takes q in reference a and recomputes it to reference b. Elements not defined in a mapping are not modified
   *
   * It does not checks for vector sizes
   */
  template <typename Vector1, typename Vector2>
  void mapReversed(const Vector1& q_a, Vector2& q_b) const
  {
    for (int i = 0; i < _map.size(); i++)
    {
      if(_map[i] != robot_class::NON_EXISTING)
        q_b[_map[i]] = q_a[i];
    }

  }

  /** @brief Maps vectors using defined map
   *
   * if map is defined as q[a] = b
   *
   * it takes q in reference b and recomputes it to reference a. Elements not defined in a mapping are not modified
   *
   * It does not checks for vector sizes
   *
   */
  template <typename Vector1, typename Vector2>
  void mapDirect(const Vector1& q_b, Vector2& q_a) const
  {
    for (int i = 0; i < _map.size(); i++)
    {
      if(_map[i] != robot_class::NON_EXISTING)
        q_a[i] = q_b[_map[i]];
    }
  }

  // select is from a to b
  template <typename Vector1, typename Vector2>
  void mapReversed(const Vector1 q_a, Vector2& q_b, const mwoibn::VectorBool& select) const
  {
    for (int i = 0; i < _map.size(); i++)
    {
      if(_map[i] != robot_class::NON_EXISTING && select[i])
        q_b[_map[i]] = q_a[i];
    }
  }

  // select is from a to b
  template <typename Vector1, typename Vector2>
  void mapDirect(const Vector1 q_b, Vector2& q_a,  const mwoibn::VectorBool& select) const
  {
    for (int i = 0; i < _map.size(); i++)
    {
      if(_map[i] != robot_class::NON_EXISTING && select[i])
        q_a[i] = q_b[_map[i]];
    }
  }

  // select is from a to b
  template <typename Vector1, typename Vector2>
  void mapReversed(const Vector1 q_a, Vector2& q_b, const mwoibn::VectorInt& select) const
  {
    for (int i = 0; i < select.size(); i++)
    {
      int j = select[i];
      if(_map[j] != robot_class::NON_EXISTING)
        q_b[_map[j]] = q_a[j];
    }
  }

  // select is from a to b
  template <typename Vector1, typename Vector2>
  void mapDirect(const Vector1 q_b, Vector2& q_a, const mwoibn::VectorInt& select) const
  {
    for (int i = 0; i < select.size(); i++)
    {
      int j = select[i];
      if(_map[j] != robot_class::NON_EXISTING )
        q_a[j] = q_b[_map[j]];
    }
  }

  double getDofsReversed() const { return _reversed.size(); }

 // I need selectors here

  // VectorBool find();

protected:
  VectorInt _reversed;
};
} // namespace package
} // namespace library
#endif // MAP_H
