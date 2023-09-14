#include<simrad_ek80/udp/connection.h>
#include <cstring>
#include <cmath>
#include <poll.h>
#include <iostream>

namespace simrad
{

Connection::Connection()
{
}

Connection::~Connection()
{
}

void Connection::connect(const sockaddr_in& address, std::string uname, std::string pwd)
{
  setRemoteAddress(address);

  packet::DisconnectRequest disconnect;
  disconnect.setNamePwd(uname,pwd);

  std::vector<uint8_t> disconnect_packet(reinterpret_cast<const uint8_t*>(&disconnect), reinterpret_cast<const uint8_t*>(&disconnect)+sizeof(packet::DisconnectRequest));

  setFinalizePacket(disconnect_packet);

  packet::ConnectRequest request;
  request.setNamePwd(uname,pwd);

  std::vector<uint8_t> packet(reinterpret_cast<const uint8_t*>(&request), reinterpret_cast<const uint8_t*>(&request)+sizeof(packet::ConnectRequest));
  sendPacket(packet);

  std::unique_lock<std::mutex> lock(clientIDMutex_);
  while(clientID_.empty())
    clientIDAvailable_.wait(lock);
  std::cerr << "Connected. Client ID: " << clientID_ << std::endl;
}

std::string Connection::getID()
{
  return clientID_;
}

int Connection::getRID()
{
  return reqID_++;
}

std::string Connection::sendRequest(const std::string& req, int request_id)
{
  std::vector< packet::Request > request;
  int parts = (int)ceil(req.size()/1400.0);
  for(int i = 0; i < parts; ++i)
  {
    packet::Request r;
    int count = req.size()+1-(i*1400);
    if(count > 1400)
      count = 1400;
    memcpy(r.MsgRequest,req.substr(i*1400).c_str(),count);
    request.push_back(r);
  }
  std::vector<std::vector<uint8_t> > request_packets;
  {
    std::lock_guard<std::mutex> lock(nextSeqNoMutex_);
    for(int i = 0; i < request.size(); ++i)
    {
      std::string msgctrl = std::to_string(nextSeqNo_) + "," + std::to_string(i+1)+","+std::to_string(request.size());
      strcpy(request[i].MsgControl, msgctrl.substr(0,21).c_str());
      request_packets.push_back(std::vector<u_int8_t>(reinterpret_cast<uint8_t*>(&request[i]), reinterpret_cast<uint8_t*>(&request[i])+sizeof(packet::Request)));
      nextSeqNo_ += 1;
    }
    sendPackets(request_packets);
  }
  
  std::unique_lock<std::mutex> res_lock(responsesMutex_);
  while(pendingResponses_.find(request_id) == pendingResponses_.end())
    responsesAvailable_.wait(res_lock);
  return pendingResponses_[request_id];
}

void Connection::receivePacket(const std::vector<uint8_t>& packet)
{

  packet::ConnectResponse* connectResponse = (packet::ConnectResponse*)&packet.front();
  packet::AliveReport* server_alive = (packet::AliveReport*)&packet.front();
  packet::RequestResponse* response = (packet::RequestResponse*)&packet.front();
  packet::Retransmit* rtr = (packet::Retransmit*)&packet.front();
        
  if (*connectResponse)
  {
    auto result = connectResponse->getResult();
      
    if (result.find("ClientID") == result.end())
      throw(Exception("ClientID not found in connection response. "+std::string(connectResponse->MsgResponse) ));

    std::unique_lock<std::mutex> lock(clientIDMutex_);
    clientID_ = result["ClientID"];
    clientIDAvailable_.notify_all();
  }
  else if(*server_alive)
  {
    packet::AliveReport alive;
    std::lock_guard<std::mutex> lock(nextSeqNoMutex_);
    alive.setInfo(clientID_,nextSeqNo_);
    sendPacket(std::vector<uint8_t>(reinterpret_cast<uint8_t*>(&alive), reinterpret_cast<uint8_t*>(&alive)+sizeof(alive)));
  }
  else if(*rtr)
  {
    std::cerr << "retransmit request from server " << rtr->MsgControl << std::endl;
  }
  else if (*response)
  {
    std::unique_lock<std::mutex> lock(responsesMutex_);
    std::string msgCtrl(response->MsgControl);
    lastServerSeqNo_ = std::atoi(msgCtrl.substr(0,msgCtrl.find(",")).c_str());
    pendingResponses_[response->getRequestID()] = response->MsgResponse;
    responsesAvailable_.notify_all();
  }
}

} // namespace simrad
