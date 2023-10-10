#ifndef SIMRAD_EK80_UDP_SOCKET_H
#define SIMRAD_EK80_UDP_SOCKET_H

#include <thread>
#include <mutex>
#include <queue>
#include <vector>
#include <functional>
#include <netinet/in.h>

namespace simrad
{

/// Establishes a threaded UDP socket.
class UDPSocket
{
public:
  UDPSocket();
  ~UDPSocket();
    
  int getPort() const;
  int remotePort() const;
protected:
  virtual void receivePacket(const std::vector<uint8_t>& packet) = 0;
  void sendPacket(const std::vector<uint8_t>& packet);
  void sendPackets(const std::vector<std::vector<uint8_t> >& packets);
  void setRemoteAddress(const sockaddr_in& address);
  void setFinalizePacket(const std::vector<uint8_t>& packet);
private:
  int socket_;
  int port_;
  sockaddr_in remote_address_;

  std::vector<std::vector<uint8_t> > outgoing_packets_;
  std::mutex outgoing_packets_mutex_;

  // optional packet to send before closing connection
  std::vector<uint8_t> finalize_packet_;

  std::thread receiver_thread_;
  bool exitThread_;
  std::mutex exitThreadMutex_;

  void receiverThread();
};

} // namespace simrad

#endif

