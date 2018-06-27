#ifndef __MWOIBN_UTILS_STD_UTILS__MAP_H
#define __MWOIBN_UTILS_STD_UTILS__MAP_H


#include <map>

/* @brief provides key and value iterators over std::maps
 *
 * thanks to https://stackoverflow.com/questions/1443793/iterate-keys-in-a-c-map/35262398#35262398
 */

template<typename Key, typename Value>
using Map = std::map<Key, Value>;

template<typename Key, typename Value>
using MapIterator = typename Map<Key, Value>::iterator;

template<typename Key, typename Value>
class MapKeyIterator : public MapIterator<Key, Value> {

public:

MapKeyIterator ( ) : MapIterator<Key, Value> ( ) {
};
MapKeyIterator ( MapIterator<Key, Value> it_ ) : MapIterator<Key, Value> ( it_ ) {
};

Key *operator -> ( ) {
        return ( Key * const ) &( MapIterator<Key, Value>::operator -> ( )->first );
}
Key operator * ( ) {
        return MapIterator<Key, Value>::operator * ( ).first;
}
};

template<typename Key, typename Value>
class MapValueIterator : public MapIterator<Key, Value> {

public:

MapValueIterator ( ) : MapIterator<Key, Value> ( ) {
};
MapValueIterator ( MapIterator<Key, Value> it_ ) : MapIterator<Key, Value> ( it_ ) {
};

Value *operator -> ( ) {
        return ( Value * const ) &( MapIterator<Key, Value>::operator -> ( )->second );
}
Value operator * ( ) {
        return MapIterator<Key, Value>::operator * ( ).second;
}
};

#endif
