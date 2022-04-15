#include<simrad_ek80/connection.h>
#include <cstring>
#include <cmath>
#include <poll.h>
#include <iostream>

namespace simrad
{

Connection::Connection(const sockaddr_in& address, std::string uname, std::string pwd)
  :sendRequest_(false)
{
  memcpy(&address_, &address, sizeof(address_));
  packet::ConnectRequest request;
  request.setNamePwd(uname,pwd);

  socket_ = socket(AF_INET, SOCK_DGRAM, 0);
  if(socket_ < 0)
    throw(Exception("Error creating socket"));
    
  sockaddr_in bind_address; // address to listen on
  memset((char *)&bind_address, 0, sizeof(bind_address));
  bind_address.sin_family = AF_INET;
  bind_address.sin_addr.s_addr = htonl(INADDR_ANY);
  bind_address.sin_port = 0;
    
  if(bind(socket_, (sockaddr*)&bind_address, sizeof(bind_address)) < 0)
    throw("Error binding socket");

  timeval socket_timeout;
  socket_timeout.tv_sec = 10;
  socket_timeout.tv_usec = 0;
  if(setsockopt(socket_, SOL_SOCKET, SO_RCVTIMEO, &socket_timeout, sizeof(socket_timeout)) < 0)
    throw Exception("Error setting socket timeout");

  sendto(socket_, &request, sizeof(request), 0, (sockaddr*)&address, sizeof(address));

  std::vector<uint8_t> responsePacket;
  responsePacket.resize(sizeof(packet::Response));
  packet::Response *response = (packet::Response*)&responsePacket.front();

  int len_received = 0;
  while(len_received != sizeof(packet::Response))
  {
    len_received = recv(socket_, responsePacket.data(), responsePacket.size(),0);
    if (len_received < 0)
      throw Exception(std::strerror(errno));
  }
    
  if(!(bool)(*response))
    throw(Exception("Invalid response received"));

  auto result = response->getResult();
  // for (auto kv: result)
  //   std::cout << kv.first << "|" << kv.second << "|" << std::endl;
    
  if (result.find("ClientID") == result.end())
    throw(Exception("ClientID not found in connection response. "+std::string(response->MsgResponse) ));
    
  clientID_ = result["ClientID"];
  nextSeqNo_ = 1;
  lastServerSeqNo_ = 0;
  reqID_ = 1;
  disconnect_.setNamePwd(uname,pwd);
  exitThread_ = false;
  connectionThread_ = std::move(std::thread(std::ref(*this)));
}

Connection::~Connection()
{
  {
    std::lock_guard<std::mutex> lock(exitThreadMutex_);
    exitThread_ = true;
  }
  connectionThread_.join();
}

std::string Connection::getID()
{
  return clientID_;
}

std::string Connection::sendRequest(const std::string& req)
{
  std::unique_lock<std::mutex> req_lock(requestMutex_);
  std::unique_lock<std::mutex> res_lock(responseMutex_);
  pendingResponse_ = "";
  
  int parts = (int)ceil(req.size()/1400.0);
  for(int i = 0; i < parts; ++i)
  {
    packet::Request r;
    int count = req.size()+1-(i*1400);
    if(count > 1400)
      count = 1400;
    memcpy(r.MsgRequest,req.substr(i*1400).c_str(),count);
    pendingRequest_.push_back(r);
  }
  
  sendRequest_ = true;
  while(pendingResponse_.empty())
      responseAvailable_.wait(res_lock);
  return pendingResponse_;
}

int Connection::getRID()
{
  return reqID_++;
}

void Connection::operator()()
{
  std::time_t lastSentRequest = 0;
  bool done = false;
  while (!done)
  {
    {
      std::unique_lock<std::mutex> lock(responseMutex_);
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
              
        packet::AliveReport* server_alive = (packet::AliveReport*)&packet.front();
        packet::Response* response = (packet::Response*)&packet.front();
        packet::Retransmit* rtr = (packet::Retransmit*)&packet.front();
              
        if(*server_alive)
        {
          std::cout << server_alive->Info << std::endl;
          packet::AliveReport alive;
          alive.setInfo(clientID_,nextSeqNo_);
          sendto(socket_, &alive, sizeof(alive), 0, (sockaddr*)&address_, sizeof(address_));
        }
        else if(*rtr)
        {
          std::cerr << "retransmit request from server" << std::endl;
        }
        else if (*response)
        {
          std::string msgCtrl(response->MsgControl);
          lastServerSeqNo_ = std::atoi(msgCtrl.substr(0,msgCtrl.find(",")).c_str());
          pendingResponse_ = response->MsgResponse;
          responseAvailable_.notify_all();
          pendingRequest_.clear();
          lastSentRequest = 0;
        }
        if(!pendingRequest_.empty())
        {
          if (lastSentRequest == 0 || lastSentRequest+5 < std::time(nullptr))
          {
            for(int i = 0; i < pendingRequest_.size(); ++i)
            {
              std::string msgctrl = std::to_string(nextSeqNo_) + "," + std::to_string(i+1)+","+std::to_string(pendingRequest_.size());
              strcpy(pendingRequest_[i].MsgControl,msgctrl.substr(0,21).c_str());
              sendto(socket_, &pendingRequest_[i], sizeof(pendingRequest_[i]), 0, (sockaddr*)&address_, sizeof(address_));
              nextSeqNo_ += 1;
            }
            lastSentRequest = std::time(nullptr);
          }
        }
      }
    }
    std::lock_guard<std::mutex> lock(exitThreadMutex_);
    if(exitThread_)
      done = true;
    else
      std::this_thread::yield();

  }
  std::cout << "simrad::Connection disconnecting" << std::endl;
  sendto(socket_, &disconnect_, sizeof(disconnect_), 0, (sockaddr*)&address_, sizeof(address_));
}

// const sockaddr_in& Connection::getLocalAddress() const
// {
//   gz4d::IPSocket::Address ret;
//   sock.GetLocalAddress(ret);
//   return ret;
// }


} // namespace simrad
