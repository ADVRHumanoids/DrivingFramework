#ifndef __MWOIBN__POINT_HANDLING__HANDLER_H
#define __MWOIBN__POINT_HANDLING__HANDLER_H

//#include "mwoibn/robot_points/point.h"

#include "mwoibn/point_handling/position.h"

namespace mwoibn
{
namespace point_handling
{

/** @brief Keeps all the contact data and probides acces to individual contacts
 * **/
 template<typename Type>
class Handler
{

public:
Handler()  {}

Handler(const Handler& other)
      : _positions(other._positions)
{
    for (auto& point : other._points)
      add(*point);
}

Handler( Handler&& other)
      : _positions(other._positions)
{
    for (auto& point : other._points)
      add(*point);
}


virtual ~Handler() { }

virtual void add(std::unique_ptr<Type> point)
{
//        add(*point);
        _points.push_back(std::move(point));

        resize();
}

virtual void add( Type&& point)
{
        _points.push_back(std::unique_ptr<Type>(new Type(point)));
        resize();
}

virtual void add(const Type& point)
{
        _points.push_back(std::unique_ptr<Type>(new Type(point)));
        resize();
}

virtual void resize(){
	double size = 0;
	for(auto& point: _points)
    		size += point->size();

	_positions = mwoibn::VectorN::Zero(size);
}


/**
 * @brief Gives number of all considered contact points
 *
 */
int size() const {
        return _points.size();
}                                               // RT?

bool remove(int i)
{
  if (i < 0 || i >= _points.size())
    return false;

  _points.erase(_points.begin() + i);
  resize();
  return true;
}


Type& point(unsigned int id)
{
  if (id < _points.size())
    return *_points.at(id);
  else
    throw std::out_of_range("Given ID is beyond a vector scope");
}


const mwoibn::VectorN& getFixed()
{

  _positions.setZero();

  unsigned int i = 0;

  for (auto& point : _points)
  {
    _positions.segment(i, point->size()) = point->getFixed();
    i += point->size();
  }

  return _positions;
}

const mwoibn::VectorN& getWorld()
{

  _positions.setZero();

  unsigned int i = 0;

  for (auto& point : _points)
  {
    _positions.segment(i, point->size()) = point->getWorld();
    i += point->size();
  }

  return _positions;
}

std::vector<mwoibn::VectorN> getPositions()
{
  std::vector<mwoibn::VectorN> positions;

  for (auto& point : _points)
  {
    positions.push_back(point->get());
  }
  return positions;
}

typename std::vector<std::unique_ptr<Type>>::iterator begin(){return _points.begin();}
typename std::vector<std::unique_ptr<Type>>::iterator end(){return _points.end();}

typename std::vector<std::unique_ptr<Type>>::const_iterator begin() const {return _points.begin();}
typename std::vector<std::unique_ptr<Type>>::const_iterator end() const {return _points.end();}

virtual Type& end(unsigned int i) {
        int idx = -i - 1;
        // std::cout << "idx\t" << idx << std::endl;
        return *(_points.end()[idx]);
}

virtual Type& operator[](int i) {
        return *_points[i];
}

protected:
std::vector<std::unique_ptr<Type> > _points;
mwoibn::VectorN _positions;



///@}
};
} // namespace package
} // namespace library
#endif // CONTACTS_H
