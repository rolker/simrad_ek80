#ifndef SIMRAD_EK80_CONNECTION_H
#define SIMRAD_EK80_CONNECTION_H

#include <thread>
#include <mutex>
#include <condition_variable>
#include <simrad_ek80/packet.h>
#include <netinet/in.h>

namespace simrad
{

class Connection
{

public:
  typedef std::shared_ptr<Connection> Ptr;

  Connection(const sockaddr_in& address, std::string uname, std::string pwd);
  
  ~Connection();

  std::string getID();
  
  std::string sendRequest(const std::string& req);

  int getRID();

  void operator()();
  

  //const sockaddr_in& getLocalAddress() const;

private:

  int socket_;
  unsigned int nextSeqNo_; /// Number that will be used for the next message to the server.
  unsigned int lastServerSeqNo_; 

  packet::DisconnectRequest disconnect_;
  std::thread connectionThread_;

  bool exitThread_;
  std::mutex exitThreadMutex_;

  sockaddr_in address_;

  std::string clientID_; /// Server supplied ID.

  int reqID_;

  std::vector<packet::Request> pendingRequest_;
  bool sendRequest_;
  std::mutex requestMutex_;

  std::string pendingResponse_;
  std::mutex responseMutex_;
  std::condition_variable responseAvailable_;
};

} // namespace simrad

#endif
