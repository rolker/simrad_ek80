#ifndef SIMRAD_EK80_CALLBACKS_H
#define SIMRAD_EK80_CALLBACKS_H

#include <functional>

namespace simrad
{

template<typename T> class Callbacks
{
public:
  using callback_type = std::function<void(T)>;

  std::shared_ptr<callback_type> addCallback(callback_type callback)
  {
    auto ret = std::make_shared<callback_type>(callback);
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    callbacks_.push_back(ret);
    return ret;
  }

protected:
  void callCallbacks(T data)
  {
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    for(auto callback: callbacks_)
      if(auto c = callback.lock())
        (*c)(data);
  }

private:            
  std::vector<std::weak_ptr<callback_type> > callbacks_;
  std::mutex callbacks_mutex_;
};

} // namespace simrad

#endif
