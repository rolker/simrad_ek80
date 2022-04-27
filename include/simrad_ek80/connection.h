#ifndef SIMRAD_EK80_CONNECTION_H
#define SIMRAD_EK80_CONNECTION_H

#include <thread>
#include <mutex>
#include <condition_variable>
#include <simrad_ek80/packet.h>
#include <simrad_ek80/udp_socket.h>

namespace simrad
{

class Connection: public UDPSocket
{

public:
  typedef std::shared_ptr<Connection> Ptr;

  Connection();
  ~Connection();

  void connect(const sockaddr_in& address, std::string uname, std::string pwd);

  std::string getID();
  
  std::string sendRequest(const std::string& req, int request_id);

  int getRID();


private:  
  void receivePacket(const std::vector<uint8_t>& packet) override;


  packet::DisconnectRequest disconnect_;
  
  bool connected_ = false;

  unsigned int nextSeqNo_ = 1;
  std::mutex nextSeqNoMutex_;

  unsigned int lastServerSeqNo_ = 0;

  std::string clientID_; /// Server supplied ID.
  std::mutex clientIDMutex_;
  std::condition_variable clientIDAvailable_;

  int reqID_ = 1;

  std::map<int, std::string> pendingResponses_;
  std::mutex responsesMutex_;
  std::condition_variable responsesAvailable_;
};

} // namespace simrad

#endif
