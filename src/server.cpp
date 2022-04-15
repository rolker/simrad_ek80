#include <simrad_ek80/server.h>
#include <sstream>
#include <cstring>
#include <arpa/inet.h>

namespace simrad
{

Server::Server()
{
  memset((char *)&address_, 0, sizeof(address_));
}

Server::Server(Server const &s):swap_(s.swap_),info_(s.info_)
{
  memcpy(&address_, &s.address_, sizeof(address_));
}

Server::Server(const sockaddr_in& address, const packet::ServerInfo2& serverInfo)
{
  if(serverInfo)
  {
    // use the port number to attempt to detect byte order. A valid IP port should fit in a 2 byte unsigned int (aka unsigned short)
    // so if the supplied value seems larger, assume byte swapping is necessary.
    swap_ = false;
    if (serverInfo.CommandPort > 0xffff)
      swap_ = true;
        
    // check if we got the swapping right
    if (swap(serverInfo.CommandPort,swap_) > 0xffff)
      throw (Exception("Problem determining if byte swapping is necessary"));
            
    memcpy(&address_, &address, sizeof(address_));
    address_.sin_port = htons(swap(serverInfo.CommandPort,swap_));
            
    info_.appType = serverInfo.ApplicationType;
    info_.appName = serverInfo.ApplicationName;
    info_.appDesc = serverInfo.ApplicationDescription;
    info_.appID = swap(serverInfo.ApplicationID,swap_);
    info_.mode = swap(serverInfo.Mode,swap_);
    info_.hostName = serverInfo.HostName;
  }
  else
      throw(Exception("Invalid packet type. Expected ServerInfo2 structure."));
}

const sockaddr_in& Server::getAddress() const
{
  return address_;
}

int Server::getID() const
{
  return info_.appID;
}

std::string Server::getType() const
{
  return info_.appType;
}

std::string Server::string() const
{
  std::stringstream s;
  s << inet_ntoa(address_.sin_addr) << ":" << ntohs(address_.sin_port) << " ";
  s << "ID:" << info_.appID << " ";
  s << "Type:" << info_.appType << " ";
  s << "Name:" << info_.appName << " ";
  s << "Mode:" << info_.mode << " ";
  s << "Desc:" << info_.appDesc << " ";
  s << "Host:" << info_.hostName;
  return s.str();
}
        
bool Server::operator==(const Server& s) const
{
  return address_.sin_addr.s_addr == s.address_.sin_addr.s_addr && address_.sin_port == s.address_.sin_port;
}

Server::operator bool() const
{
  return address_.sin_addr.s_addr != 0;
}

} // namespace simrad
