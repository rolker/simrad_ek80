#include <simrad_ek80/udp/udp_socket.h>

#include <simrad_ek80/utilities.h>
#include <cstring>
#include <poll.h>

namespace simrad
{

UDPSocket::UDPSocket()
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
  receiver_thread_ = std::move(std::thread(&UDPSocket::receiverThread,this));
}

UDPSocket::~UDPSocket()
{
  {
    std::lock_guard<std::mutex> lock(exitThreadMutex_);
    exitThread_ = true;
  }
  receiver_thread_.join();
}

int UDPSocket::getPort() const
{
  return port_;
}

void UDPSocket::sendPacket(const std::vector<uint8_t>& packet)
{
  std::lock_guard<std::mutex> lock(outgoing_packets_mutex_);
  outgoing_packets_.push_back(packet);
}

void UDPSocket::sendPackets(const std::vector<std::vector<uint8_t> >& packets)
{
  std::lock_guard<std::mutex> lock(outgoing_packets_mutex_);  
  for(auto packet: packets)
  {
    outgoing_packets_.push_back(packet);
  }
}

void UDPSocket::setRemoteAddress(const sockaddr_in& address)
{
  memcpy(&remote_address_, &address, sizeof(sockaddr_in));
}
  
void UDPSocket::setFinalizePacket(const std::vector<uint8_t>& packet)
{
  finalize_packet_ = packet;
}

void UDPSocket::receiverThread()
{
  bool done = false;
  while (!done)
  {
    pollfd p;
    p.fd = socket_;
    p.events = POLLIN;
    int ret = poll(&p, 1, 0);
    if(ret < 0)
      perror(nullptr);
    if(ret > 0 && p.revents & POLLIN)
    {
      std::vector<uint8_t> packet;
      packet.resize(1500);
      int len_received = recv(socket_, packet.data(), packet.size(),0);
      if (len_received > 0)
      {
        packet.resize(len_received);
        receivePacket(packet);
      }
    }
    {
      std::lock_guard<std::mutex> lock(outgoing_packets_mutex_);
      for(auto packet: outgoing_packets_)
        sendto(socket_, packet.data(), packet.size(), 0, (sockaddr*)&remote_address_, sizeof(remote_address_));
      std::this_thread::yield();
      outgoing_packets_.clear();
    }
    std::lock_guard<std::mutex> lock(exitThreadMutex_);
    if(exitThread_)
      done = true;
    else
      std::this_thread::yield();
  }
  if(!finalize_packet_.empty())
    sendto(socket_, finalize_packet_.data(), finalize_packet_.size(), 0, (sockaddr*)&remote_address_, sizeof(remote_address_));

}

} // namespace simrad
