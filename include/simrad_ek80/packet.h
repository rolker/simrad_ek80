#ifndef SIMRAD_EK80_PACKET_H
#define SIMRAD_EK80_PACKET_H

#include <simrad_ek80/utilities.h>

namespace simrad
{
namespace packet
{

#pragma pack(1)

struct Base
{
  char Header[4];
  Base();
  operator char const*() const;
protected:
  Base(std::string type);
  bool isHeader(std::string h) const;
};

struct RequestServerInfo: public Base
{
  RequestServerInfo();
  operator bool() const;
};

/// ServerInfo2 from Simrad EM70 interface specification
struct ServerInfo2: public Base //"SI2\0"
{
  char ApplicationType[64];
  char ApplicationName[64]; // Name of the current application
  char ApplicationDescription[128]; // Description of the current application
  int32_t ApplicationID; // ID of the current application
  int32_t CommandPort; // Port number to send commands to
  int32_t Mode; // Whether the application is running against the local data source or a remote data source
  char HostName[64]; // IP address of the computer the application is running on

  operator bool() const;

};

struct ClientInfo: public Base
{
  char Info[1024];
protected:
  ClientInfo(std::string type);
  void setInfo(std::string const &info);
};

/// Connection management base
struct Connection: public ClientInfo
{
  void setNamePwd(std::string username = "Simrad", std::string password = std::string());
protected:
  Connection(std::string type);
};

struct ConnectRequest: public Connection
{
  ConnectRequest();
};

struct DisconnectRequest: public Connection
{
  DisconnectRequest();
};

/// Alive message sent from both the client and server
struct AliveReport: public ClientInfo
{
  AliveReport();
  operator bool() const;
  void setInfo(std::string clientID, int seqNo);
};

/// Response from Simrad EM70 interface specification
struct Response: public Base
{
  char Request[4];
  char MsgControl[22];
  char MsgResponse[1400];
  operator bool() const;
  std::map<std::string, std::string> getResult() const;
  
};

struct ConnectResponse: public Response
{
  operator bool() const;
  std::map<std::string, std::string> getResult() const;  
};

struct RequestResponse: public Response
{
  operator bool() const;
  unsigned int getRequestID() const;
};

/// Retransmit request from Simrad EM70
struct Retransmit: public Base
{
  char MsgControl[22];
  operator bool() const;
};

            
/// Request to server
struct Request: public Base
{
  char MsgControl[22];
  char MsgRequest[1400];
  Request();
};

/// Subscription data
struct ProcessedData: public Base
{
  int32_t SeqNo;
  int32_t SubscriptionID;
  unsigned short CurrentMsg;
  unsigned short TotalMsg;
  unsigned short NoOfBytes;
  unsigned char* data;
  operator bool() const;
};

#pragma pack()

} // namespace packet
} // namespace simrad

#endif
