#ifndef __MWOIBN__COMMON__UPDATE_MANAGER_H
#define __MWOIBN__COMMON__UPDATE_MANAGER_H


#include "mwoibn/common/types.h"

namespace mwoibn
{

namespace update
{

  typedef std::function<void()> update_function;

  class Function {
  public:
    Function(update_function function_ptr, unsigned int subscribed = 0): _current(0), _subscribed(subscribed), _function(function_ptr){}

    void count(){ _current--; }

    int state(){return _current;}

    int subscribers(){return _subscribed;}

    void subscribe(int i = 1){_subscribed += i;}
    bool unsubscribe(int i = 1){
          if (_subscribed < i) {_subscribed = 0; return false;}
          _subscribed -= i;
        return true;

          }
    void reset(){ _current = _subscribed;}

    bool done(){return _current < 1;} // it will return true also if overcalled
    void call(){
        _function();
        _current = _subscribed;
      }

    update_function& get(){return _function;}

  protected:
    int _current;
    unsigned int _subscribed;
    update_function _function;
  };

  typedef std::deque<std::shared_ptr<Function> > store;

  //template<typename Interface>
  class UpdateManager {

  public:

      std::shared_ptr<Function> signIn(update_function update){
        _update.push_back(std::make_shared<Function>(update, 0));
        return _update.back();
      }

    // void subscribe(std::vector<update_function> functions){
    //     for(auto& update_: functions)
    //         subscribe(update_);
    // }
    //
    // void subscribe(update_function update, int i = 1){
    //   if(is(update))
    //     _access(update).subscribe(i);
    //   else
    //     _update.push_back(Function(update, i));
    // }
    //
    // bool unsubscribe(update_function update, int i = 1){
    //       if(!is(update) || !_access(update).subscribers()) return false;
    //
    //       _access(update).unsubscribe(i);
    //       return true;
    // }

    void update() {
      for(auto& function: _update)
          function->call();
    }

    void reset() {
      for(auto& function: _update)
          function->reset();
    }

    bool done(){
      for(auto& function: _update){
          if (!function->done()) return false;
      }
      return true;
    }
    //
    // int state(update_function function){
    //   if(!is(function)) return 0;
    //
    //   return _access(function).state();
    // }
    //
    // int subscribers(update_function function){
    //   if(!is(function)) return 0;
    //
    //   return _access(function).subscribers();
    // }


    typename store::iterator begin(){return _update.begin();}
    typename store::iterator end(){return _update.end();}

    typename store::const_iterator begin() const {return _update.begin();}
    typename store::const_iterator end() const {return _update.end();}

    //
    // std::function< int() > counter(update_function& update){
    //       return std::bind(&Function::count, &_access(update));
    //     }
    //
    // bool is(update_function& update) {
    //       auto it =
    //           std::find_if(_update.begin(), _update.end(), [&](Function function_)
    //                        {
    //                          _getAddress(update);
    //                          _getAddress(function_.get());
    //                          return true;
    //                        });
    //
    //       return (it != _update.end());
    // }

  protected:
     store _update;

     // Function& _access(update_function& update) {
     //       const auto& it =
     //           std::find_if(_update.begin(), _update.end(), [&](Function function_)
     //                        {
     //                          _getAddress(update);
     //                          _getAddress(function_.get());
     //                          return true;
     //                        });
     //
     //       if (it == _update.end())
     //         throw std::invalid_argument(__PRETTY_FUNCTION__ + std::string(": Requested access to unknown update function"));
     //
     //       //return it->get();
     //       return *it;
     // }



       // template<typename T, typename... U>
       // size_t _getAddress(std::function<T(U...)> f) {
       //     typedef T(fnType)(U...);
       //     fnType ** fnPointer = f.template target<fnType*>();
       //     return (size_t) *fnPointer;
       // }
       //
       // size_t _getAddress(std::function<void (void)> function) {
       // typedef void (fnType)(void);
       // fnType ** fnPointer = function.target<fnType *>();
       // return (size_t) *fnPointer;
       //     }



  };


}
}
#endif
