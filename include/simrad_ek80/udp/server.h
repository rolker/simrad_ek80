#ifndef SIMRAD_EK80_SERVER_H
#define SIMRAD_EK80_SERVER_H

#include <simrad_ek80/udp/packet.h>
#include <netdb.h>

namespace simrad
{

/// Represents a Simrad server on the network.
/// Contains the necessary info to create a Client connection to the represented server.
class Server
{
public:
  Server();
  Server(Server const &s);
  Server(const sockaddr_in& address, const packet::ServerInfo2& serverInfo);

  const sockaddr_in& getAddress() const;

  std::string getApplicationType() const;
  std::string getApplicationName() const;
  std::string getApplicationDescription() const;
  int getApplicationID() const;
  int getCommandPort() const;
  int getMode() const;
  std::string getHostName() const;

  std::string string() const;

  bool operator==(const Server& s) const;
  operator bool() const;
private:
  sockaddr_in address_;
  bool swap_=false; /// True if byte swapping is needed.

  struct
  {
    std::string appType; ///< Application type as supplied by info request
    std::string appName; ///< Application type as supplied by info request
    std::string appDesc; ///< Application type as supplied by info request
    int appID; ///< Application ID as supplied by info request
    int mode; ///< Mode indicating if application is running against remote or local data source, supplied by info request
    std::string hostName; ///< Host name as supplied by info request
  } info_;
        
};

} // namespace simrad

#endif
