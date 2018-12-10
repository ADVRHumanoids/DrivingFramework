#ifndef __MWOIBN__COMMUNICATION_MODULES__COMMUNICATIONS_H
#define __MWOIBN__COMMUNICATION_MODULES__COMMUNICATIONS_H

#include "mwoibn/communication_modules/communication_base.h"
#include <memory>

namespace mwoibn {

namespace communication_modules {

class Communications{


public:
   Communications(){}
   Communications(Communications&& other) {
     for(auto& module: other._modules)
       _modules[module.first] = std::move(module.second);
   }

   virtual ~Communications(){}


   template<typename Type>
   void add(std::unique_ptr<Type> module, std::string name){
     if(_modules.count("name"))
     throw std::invalid_argument(std::string(__PRETTY_FUNCTION__) + std::string(": Communication module ") + std::string(name) + " has already been registered in the stack." );

      _modules[name] = std::move(module);
   }

   template<typename Type>
   void add(Type&& module, std::string name){
     if(_modules.count("name"))
     throw std::invalid_argument(std::string(__PRETTY_FUNCTION__) + std::string(": Communication module ") + std::string(name) + " has already been registered in the stack." );

      _modules[name] = std::unique_ptr<CommunicationBase>(new Type(module));
   }

   template<typename Type>
   void add(Type& module, std::string name){
     if(_modules.count("name"))
     throw std::invalid_argument(std::string(__PRETTY_FUNCTION__) + std::string(": Communication module ") + std::string(name) + " has already been registered in the stack." );
      _modules[name] = std::unique_ptr<CommunicationBase>(new Type(module));
   }
   //
   // template<typename Type>
   // void add(Type* module, std::string name = ""){
   //    _modules[name] = std::unique_ptr<CommunicationBase>(new Type(module));
   // }

   // virtual bool remove(int i){
   //     if (i < 0 || i >= _feedbacks.size())
   //       return false;
   //
   //     _feedbacks.erase(_feedbacks.begin() + i);
   //     _names.erase(_names.begin() + i);
   //
   //     return true;
   // }

   bool run()
   {

     bool success = true;
     for (auto& module : _modules)
       success = module.second->run() && success;

     return success;
   }

   bool initialized(){
     bool success = true;
     for (auto& module : _modules)
       success = module.second->initialized() && success;

     return success;
   }

   bool reset(){
     bool success = true;

     for (auto& module : _modules)
       success = module.second->reset() && success;

     return success;
   }

   // mwoibn::communication_modules::BasicFeedback& feedback(unsigned int id){
   //   if (id < _feedbacks.size())
   //     return *_feedbacks.at(id);
   //   else
   //     throw std::out_of_range("Given ID is beyond a vector scope");
   // }

   // int getId(std::string name)
   // {
   //
   //   auto name_ptr =
   //       std::find_if(_names.begin(), _names.end(), [&name](std::string names)
   //                    {
   //                      return names == name;
   //                    });
   //   if (name_ptr == _names.end())
   //   {
   //     throw std::invalid_argument("Couldn't find controller " + name);
   //   }
   //
   //   return std::distance(_names.begin(), name_ptr);
   // }

   mwoibn::communication_modules::CommunicationBase& module(std::string name)
   {
     return *_modules[name];
   }

   void remove(std::string name) { _modules.erase(name); }

   bool has(std::string name){
     return _modules.count(name);
   }

   std::map<std::string, std::unique_ptr<CommunicationBase>>::iterator begin(){return _modules.begin();}
   std::map<std::string, std::unique_ptr<CommunicationBase>>::iterator end(){return _modules.end();}

   std::map<std::string, std::unique_ptr<CommunicationBase>>::const_iterator begin() const {return _modules.begin();}
   std::map<std::string, std::unique_ptr<CommunicationBase>>::const_iterator end() const {return _modules.end();}

   virtual CommunicationBase& operator[](std::string name) {
           return *_modules[name];
   }


protected:
   std::map<std::string, std::unique_ptr<mwoibn::communication_modules::CommunicationBase>> _modules;
};
}
}

#endif // FEEDBACKS_H
