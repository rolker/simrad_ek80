#include <simrad_ek80/subscription_port.h>

#include <sys/socket.h>
#include <netdb.h>
#include <simrad_ek80/utilities.h>
#include <cstring>
#include <poll.h>
#include <iostream>

namespace simrad
{

SubscriptionPort::SubscriptionPort()
{
  socket_ = socket(AF_INET, SOCK_DGRAM, 0);
  if(socket_ < 0)
    throw(Exception("Error creating socket"));
    
  sockaddr_in bind_address; // address to listen on
  memset((char *)&bind_address, 0, sizeof(bind_address));
  bind_address.sin_family = AF_INET;
  bind_address.sin_addr.s_addr = htonl(INADDR_ANY);
  bind_address.sin_port = 0;
    
  if(bind(socket_, (sockaddr*)&bind_address, sizeof(bind_address)) < 0)
    throw(Exception("Error binding socket"));

  socklen_t bind_address_len = sizeof(bind_address);
  if(getsockname(socket_, (struct sockaddr*) &bind_address, &bind_address_len))
    throw(Exception("Error reading socket address"));

  port_ = ntohs(bind_address.sin_port);

  exitThread_ = false;
  receiver_thread_ = std::move(std::thread(&SubscriptionPort::ReceiverThread,this));
}

SubscriptionPort::~SubscriptionPort()
{
  {
    std::lock_guard<std::mutex> lock(exitThreadMutex_);
    exitThread_ = true;
  }
  receiver_thread_.join();
}

int SubscriptionPort::getPort() const
{
  return port_;
}

void SubscriptionPort::addCallback(std::function<void (const std::vector<uint8_t>&)> callback)
{
  std::lock_guard<std::mutex> lock(callbacks_mutex_);
  callbacks_.push_back(callback);
}

void SubscriptionPort::ReceiverThread()
{
  bool done = false;
  while (!done)
  {
    pollfd p;
    p.fd = socket_;
    p.events = POLLIN;
    int ret = poll(&p, 1, 200);
    if(ret < 0)
      perror(nullptr);
    if(ret > 0 && p.revents & POLLIN)
    {
      std::vector<uint8_t> packet;
      packet.resize(1500);
      int len_received = recv(socket_, packet.data(), packet.size(),0);
      if (len_received > 0)
      {
        //std::cout << "SubscriptionPort::ReceiverThread: " << len_received << " bytes received on port " << port_ << std::endl;
        packet.resize(len_received);
        std::lock_guard<std::mutex> lock(callbacks_mutex_);
        for(auto callback: callbacks_)
          callback(packet);
      }
    }
    std::lock_guard<std::mutex> lock(exitThreadMutex_);
    if(exitThread_)
      done = true;
    else
      std::this_thread::yield();
  }
}


} // namespace simrad
