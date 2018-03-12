#ifndef ROBOT_CLASS_FEEDBACKS_H
#define ROBOT_CLASS_FEEDBACKS_H

#include "mwoibn/communication_modules/basic_feedback.h"
#include <memory>

namespace mwoibn {

namespace robot_class {

class Feedbacks{

public:
   Feedbacks(){}
   virtual ~Feedbacks(){}

   virtual void add(std::unique_ptr<mwoibn::communication_modules::BasicFeedback> feedback, std::string name = ""){
      _feedbacks.push_back(std::move(feedback));
      _names.push_back(name);
   }

   virtual bool remove(int i){
       if (i < 0 || i >= _feedbacks.size())
         return false;

       _feedbacks.erase(_feedbacks.begin() + i);
       _names.erase(_names.begin() + i);

       return true;
   }

   bool get()
   {

     bool success = true;
     for (auto& feedback : _feedbacks)
       success = feedback->get() && success;

     return success;
   }

   bool initialized(){
     bool success = true;
     for (auto& feedback : _feedbacks)
       success = feedback->initialized() && success;

     return success;
   }

   void reset(){
     for (auto& feedback : _feedbacks)
       feedback->reset();

     return;
   }

   mwoibn::communication_modules::BasicFeedback& feedback(unsigned int id){
     if (id < _feedbacks.size())
       return *_feedbacks.at(id);
     else
       throw std::out_of_range("Given ID is beyond a vector scope");
   }

   int getId(std::string name)
   {

     auto name_ptr =
         std::find_if(_names.begin(), _names.end(), [&name](std::string names)
                      {
                        return names == name;
                      });
     if (name_ptr == _names.end())
     {
       throw std::invalid_argument("Couldn't find controller " + name);
     }

     return std::distance(_names.begin(), name_ptr);
   }

   mwoibn::communication_modules::BasicFeedback& feedback(std::string name)
   {
     return *_feedbacks.at(getId(name));
   }

   void remove(std::string name) { remove(getId(name)); }


protected:
   std::vector<std::unique_ptr<mwoibn::communication_modules::BasicFeedback>>
       _feedbacks;
   std::vector<std::string> _names;
};
}
}

#endif // FEEDBACKS_H
