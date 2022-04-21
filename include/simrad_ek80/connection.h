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

  Connection();
  ~Connection();

  void connect(const sockaddr_in& address, std::string uname, std::string pwd);

  std::string getID();
  
  std::string sendRequest(const std::string& req, int request_id);

  int getRID();

  void operator()();
  

  //const sockaddr_in& getLocalAddress() const;

private:
  int socket_;

  packet::DisconnectRequest disconnect_;
  
  std::thread connectionThread_;
  bool connected_ = false;

  bool exitThread_;
  std::mutex exitThreadMutex_;

  sockaddr_in address_;

  std::string clientID_; /// Server supplied ID.

  int reqID_;

  std::vector<std::vector<packet::Request> > pendingRequests_;
  std::mutex requestsMutex_;

  std::map<int, std::string> pendingResponses_;
  std::mutex responsesMutex_;
  std::condition_variable responsesAvailable_;
};

} // namespace simrad

#endif
