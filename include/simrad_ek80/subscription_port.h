#ifndef SIMRAD_EK80_SUBSCRIPTION_PORT_H
#define SIMRAD_EK80_SUBSCRIPTION_PORT_H

#include <thread>
#include <mutex>
#include <queue>
#include <vector>
#include <functional>

namespace simrad
{

/// Establishes UDP socket and pushes received packets to a queue.
class SubscriptionPort
{
public:
  typedef std::shared_ptr<SubscriptionPort> Ptr;

  SubscriptionPort();
  ~SubscriptionPort();
    
  int getPort() const;
  void addCallback(std::function<void (const std::vector<uint8_t>&)> callback);

private:
  int socket_;
  int port_;

  std::vector<std::function<void (const std::vector<uint8_t>&)> > callbacks_;

  std::thread receiver_thread_;
  bool exitThread_;
  std::mutex exitThreadMutex_;

  void ReceiverThread();
};

} // namespace simrad

#endif

