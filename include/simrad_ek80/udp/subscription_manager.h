#ifndef SIMRAD_EK80_SUBSCRIPTION_MANAGER_H
#define SIMRAD_EK80_SUBSCRIPTION_MANAGER_H

#include <simrad_ek80/udp/request.h>
#include <simrad_ek80/udp/connection.h>
#include <simrad_ek80/udp/udp_socket.h>
#include <simrad_ek80/udp/subscription.h>

namespace simrad
{

class SubscriptionManager: public UDPSocket
{
public:
  typedef std::shared_ptr<SubscriptionManager> Ptr;

  SubscriptionManager(Connection::Ptr& connection);
  ~SubscriptionManager();

  void subscribe(Subscription::Ptr subscription);
                
  void update(Subscription::Ptr& subscription);
                
  void unsubscribe(Subscription::Ptr& subscription);
                
private:
  void receivePacket(const std::vector<uint8_t>& packet) override;

  Connection::Ptr connection_;

  std::mutex subscriptions_mutex_;
  std::map<int, Subscription::Ptr> subscriptions_;

  std::map<int32_t, std::vector< std::vector<uint8_t> > > partial_packets_;
};

} // namespace simrad

#endif

