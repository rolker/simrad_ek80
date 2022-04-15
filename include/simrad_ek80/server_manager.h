#ifndef SIMRAD_EK80_SERVER_MANAGER_H
#define SIMRAD_EK80_SERVER_MANAGER_H

#include <list>
#include <thread>
#include <mutex>
#include <boost/property_tree/xml_parser.hpp>
#include <simrad_ek80/server.h>

namespace simrad
{

/// Manages servers available on the network.
/// Sends a request for servers, and keeps a list of the responses.
class ServerManager
{
public:
  ServerManager(std::vector<std::string> remote_addresses = std::vector<std::string>(), uint16_t port=37655);
  ServerManager(ServerManager const &other);

  ~ServerManager();

  std::vector<Server> getList();
        
  Server get(const sockaddr_in& address);
  Server get(int id);
  Server get(boost::property_tree::ptree const &xml);

private:
  uint16_t port_;
  std::vector<std::string> remote_addresses_;

  std::vector<Server> serverList_;
  std::mutex listMutex_;

  std::thread searchThread_;
  bool exitThread_;
  std::mutex exitThreadMutex_;

  void searchThread();
  void launchSearchThread();
    
};

} // namespace simrad

#endif
