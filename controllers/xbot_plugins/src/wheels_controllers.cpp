#include "mgnss/xbot_plugins/wheels_controllers.h"


bool mgnss::xbot_plugins::WheelsControllerExtend::init_control_plugin(XBot::Handle::Ptr handle){
        mgnss::plugins::XbotBase::init_control_plugin(handle);
        return true;
}

void mgnss::xbot_plugins::WheelsControllerExtend::on_start(double time)
{
        mgnss::plugins::XbotBase::on_start(time);

}

void mgnss::xbot_plugins::WheelsControllerExtend::control_loop(double time)
{

        _begin = std::chrono::high_resolution_clock::now();

        _valid = _robot_ptr->get();

        if (!_valid)
                return;

        _robot_ptr->updateKinematics();

        if (!_initialized)
        {
                _setRate(_robot_ptr->rate());
                _valid = _robot_ptr->feedbacks.reset();
                if(_valid) {
                        _controller_ptr->init();
                        _initialized = true;
                }
        }

        _controller_ptr->update();
        _controller_ptr->send();

        _end = std::chrono::high_resolution_clock::now();
       _logger_ptr->add("update", std::chrono::duration_cast<std::chrono::microseconds>((_end-_begin)).count());

        _controller_ptr->log(*_logger_ptr.get(), time-_start);
        _logger_ptr->write();

  }

//}
/*
   bool mgnss::xbot_plugins::WheelsV2::close() {

   //    _logger->flush();

   //    file.flush();
   //    file.close();

   //	std::cout << "file closed" << std::endl;
   return true; }
 */
