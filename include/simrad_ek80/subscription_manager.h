#ifndef SIMRAD_EK80_SUBSCRIPTION_MANAGER_H
#define SIMRAD_EK80_SUBSCRIPTION_MANAGER_H

#include <simrad_ek80/request.h>
#include <simrad_ek80/connection.h>
#include <simrad_ek80/subscription_port.h>
#include <simrad_ek80/subscription.h>

namespace simrad
{

class SubscriptionManager
{
public:
  typedef std::shared_ptr<SubscriptionManager> Ptr;

  SubscriptionManager(Connection::Ptr& connection);
  ~SubscriptionManager();

  void subscribe(Subscription::Ptr& subscription);
                
  void update(Subscription::Ptr& subscription);
                
  void unsubscribe(Subscription::Ptr& subscription);
                
  void packet_callback(const std::vector<uint8_t>& packet);
private:
  Connection::Ptr connection_;

  SubscriptionPort::Ptr port_;

  std::mutex subscriptions_mutex_;
  std::map<int, Subscription::Ptr> subscriptions_;

  std::map<int32_t, std::vector< std::vector<uint8_t> > > partial_packets_;
};

} // namespace simrad

#endif

