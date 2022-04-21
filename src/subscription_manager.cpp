#include <simrad_ek80/subscription_manager.h>
#include <iostream>

namespace simrad
{

SubscriptionManager::SubscriptionManager(Connection::Ptr& connection)
  :connection_(connection)
{
  port_ =  std::make_shared<SubscriptionPort>();
  port_->addCallback([this](const std::vector<uint8_t>& packet){this->packet_callback(packet);});
}
                
SubscriptionManager::~SubscriptionManager()
{
  for(auto subscription: subscriptions_)
  {
    Request req (connection_,"RemoteDataServer","Unsubscribe");
    req.addArgument("subscriptionID", std::to_string(subscription.first));
    req.getResponse();
  }
}

void SubscriptionManager::subscribe(Subscription::Ptr& subscription)
{
  Request req(connection_, "RemoteDataServer", "Subscribe");
  req.addArgument("dataRequest", subscription->subscribeString());
  std::cout << "dataRequest: " << subscription->subscribeString() << std::endl;
  req.addArgument("requestedPort", std::to_string(port_->getPort()));

  std::cout << "SubscriptionManager::subscribe port: " << port_->getPort() << std::endl;

  std::lock_guard<std::mutex> lock(subscriptions_mutex_);
    
  Response r(req.getResponse());
  if(r.getErrorCode())
      throw(Exception(r.getErrorMessage()));
  std::stringstream ss(r.getArgument("subscriptionID"));
  int id;
  ss >> id;
  subscription->setID(id);
  subscriptions_[id] = subscription;
}

void SubscriptionManager::unsubscribe(Subscription::Ptr& subscription)
{
  Request req(connection_, "RemoteDataServer", "Unsubscribe");
  req.addArgument("subscriptionID", std::to_string(subscription->getID()));
  Response r(req.getResponse());
  if(r.getErrorCode())
    throw(Exception(r.getErrorMessage()));
}

void SubscriptionManager::packet_callback(const std::vector<uint8_t>& packet)
{
  const packet::ProcessedData *data = reinterpret_cast<const packet::ProcessedData *>(packet.data());
  if(*data)
  {
    if(data->TotalMsg > 1)
    {
      int message_id = data->SeqNo - data->CurrentMsg + 1;
      if(partial_packets_[message_id].empty())
      {
        partial_packets_[message_id].resize(data->TotalMsg);
        //std::cout << "got first of " << data->TotalMsg << " for message_id " << message_id  << " from SeqNo " << data->SeqNo << std::endl;
      }
      auto data_it = packet.begin() + sizeof(packet::ProcessedData) - sizeof(unsigned char *);
      partial_packets_[message_id][data->CurrentMsg-1].insert(partial_packets_[message_id][data->CurrentMsg-1].end(), data_it, packet.end());

      
      bool complete = true;
      for(const auto& fragment: partial_packets_[message_id])
        if(fragment.empty())
        {
          complete = false;
          break;
        }
      if(complete)
      {
        //std::cout << "assembling SeqNo " << data->SeqNo << std::endl;
        auto full_packet = std::make_shared<std::vector<uint8_t> >();
        for(auto fragment: partial_packets_[message_id])
        {
          full_packet->insert(full_packet->end(), fragment.begin(), fragment.end());
        }
        partial_packets_.erase(message_id);
        std::lock_guard<std::mutex> lock(subscriptions_mutex_);
        if(subscriptions_[data->SubscriptionID])
          subscriptions_[data->SubscriptionID]->addData(full_packet);
      }
    }
    else
    {
      auto data_ptr = std::make_shared<std::vector<uint8_t> >(data->data,&data->data[data->NoOfBytes]);
      std::lock_guard<std::mutex> lock(subscriptions_mutex_);
      if(subscriptions_[data->SubscriptionID])
        subscriptions_[data->SubscriptionID]->addData(data_ptr);
    }
  }
  else
  {
      std::cerr << "Not the subscription packet I was expecting... " << std::endl;
  }
}


void SubscriptionManager::update(Subscription::Ptr& subscription)
{
  Request req(connection_, "RemoteDataServer", "ChangeSubscription");
  req.addArgument("subscriptionID", std::to_string(subscription->getID()));
  req.addArgument("dataRequest", subscription->subscribeString());
                  
  std::lock_guard<std::mutex> lock(subscriptions_mutex_);

  Response r(req.getResponse());
  if(r.getErrorCode())
    throw(Exception(r.getErrorMessage()));
}

} // namespace simrad
