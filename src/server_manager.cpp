#include <simrad_ek80/server_manager.h>

#include <sys/socket.h>
#include <netdb.h>
#include <iostream>

namespace simrad
{

ServerManager::ServerManager(std::vector<std::string> remote_addresses, uint16_t port)
  :port_(port), remote_addresses_(remote_addresses)
{
  launchSearchThread();
}

ServerManager::ServerManager(const ServerManager& other)
  :port_(other.port_), remote_addresses_(other.remote_addresses_)
{
  launchSearchThread();
}

ServerManager::~ServerManager()
{
  {
    std::lock_guard<std::mutex> lock(exitThreadMutex_);
    exitThread_ = true;
  }
  searchThread_.join();
}

std::vector<Server> ServerManager::getList()
{
  std::lock_guard<std::mutex> lock(listMutex_);
  return serverList_;
}

Server ServerManager::get(const sockaddr_in& address)
{
  auto list = getList();
  for(auto s: list)
    if(s.getAddress().sin_addr.s_addr == address.sin_addr.s_addr)
      return s;
  return Server();
}

Server ServerManager::get(int id)
{
  auto list = getList();
  for(auto s: list)
    if(s.getID() == id)
      return s;
  return Server();
}
        
// Server ServerManager::get(boost::property_tree::ptree const &xml)
// {
//   int id=0;
//   bool hasID = false;
//   if(xml.count("Client.<xmlattr>.id"))
//   {
//       hasID = true;
//       id = xml.get("Client.<xmlattr>.id",id);
//   }
        
//   std::string address;
//   bool hasAddress = false;
//   if(xml.count("Client.<xmlattr>.address"))
//   {
//       hasAddress = true;
//       address = xml.get("Client.<xmlattr>.address",address);
//   }
    
//   auto list = getList();
//   for(auto s: list)
//     if( (!hasID || s.getID() == id) && (!hasAddress || s.getAddress() == address))
//       return s;
//   return Server();
// }

void ServerManager::launchSearchThread()
{
  exitThread_ = false;
  searchThread_ = std::thread(&ServerManager::searchThread,this);
}

void ServerManager::searchThread()
{
  int s = socket(AF_INET, SOCK_DGRAM, 0);
  if(s < 0)
  {
    std::cerr << "Error creating socket" << std::endl;
    return;
  }
    
  sockaddr_in bind_address; // address to listen on
  memset((char *)&bind_address, 0, sizeof(bind_address));
  bind_address.sin_family = AF_INET;
  bind_address.sin_addr.s_addr = htonl(INADDR_ANY);
  bind_address.sin_port = 0;
    
  if(bind(s, (sockaddr*)&bind_address, sizeof(bind_address)) < 0)
  {
    std::cerr << "Error binding socket" << std::endl;
    return;
  }

  timeval socket_timeout;
  socket_timeout.tv_sec = 1;
  socket_timeout.tv_usec = 0;
  if(setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &socket_timeout, sizeof(socket_timeout)) 
  < 0)
  {
    std::cerr << "Error setting socket timeout" << std::endl;
    return;
  }

  int broadcast=1;
  if(setsockopt(s, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) < 0)
  {
    std::cerr << "Error setting socket to broadcast" << std::endl;
    return;
  }

  std::vector<uint32_t> remotes;
  if(!remote_addresses_.empty())
    for(auto a: remote_addresses_)
      remotes.push_back(ipAddressFromString(a));
  else
    remotes = getLocalAddresses(true);

  // for(auto a: remotes)
  //   std::cout << ipAddressToString(a) << std::endl;

  std::time_t lastBroadcast = 0;
  std::vector<uint8_t> response;
  sockaddr_in server_address;
  socklen_t server_address_length = sizeof(server_address);

  packet::RequestServerInfo infoReq;
        
  bool done = false;
  while (!done)
  {
    std::time_t now = std::time(nullptr);
    if (now > (lastBroadcast + 3))
    {
      for(auto remote_address: remotes)
      {
        sockaddr_in address;
        memset((char *)&address, 0, sizeof(address));
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = remote_address;
        address.sin_port = htons(port_);
        if(sendto(s, &infoReq, sizeof(infoReq), 0, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0)
        {
          std::cerr << "Error sending info request to " << ipAddressToString(remote_address) << std::endl;
        }
      }
      lastBroadcast = now;
    }
            
    response.resize(1500);
    int receive_length = recvfrom(s, response.data(), response.size(), 0, (sockaddr*)&server_address, &server_address_length);
    if(receive_length < 0 && errno != EAGAIN)
    {
      std::cerr << "Error receiving response: " << errno << std::endl;
      perror(nullptr);
    }
    if(receive_length > 0)
    {
      packet::ServerInfo2 *si = (packet::ServerInfo2*)&response.front();
      if (*si)
      {
        Server s(server_address, *si);
        std::lock_guard<std::mutex> lock(listMutex_);
        bool exists = false;
        for(auto server: serverList_)
        {
          if (server == s)
          {
              exists = true;
              break;
          }
        }
        if(!exists)
          serverList_.push_back(s);
      }
    }
    std::lock_guard<std::mutex> lock(exitThreadMutex_);
    if(exitThread_)
      done = true;
  }
}


} // namespace simrad
