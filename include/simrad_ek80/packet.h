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

/// Parameter updates

struct RecordHeader
{
  unsigned short recID;
  unsigned short recLen;
  RecordHeader* next(){return reinterpret_cast<RecordHeader*>((char*)(this)+recLen);}
};

struct ParameterMessageHeader
{
  uint16_t currentMsg;
  uint16_t totalMsg;
  int32_t seqNo;
  int32_t msgID;
  RecordHeader* records() {return reinterpret_cast<RecordHeader*>(this+1);}
};


struct ParameterDefinitionValue
{
    int32_t len;
    char* value() {return (char*)(this)+sizeof(len);}
    int Size(){return len+sizeof(len);}
};

struct ParameterDefinition
{
  int32_t cookie;
  uint64_t timeStamp;
  ParameterDefinitionValue* value() {return reinterpret_cast<ParameterDefinitionValue*>((char*)(this+1)-sizeof(int32_t));} ;
  int32_t unused;
  int Size(){return sizeof(ParameterDefinition) + value()->Size();}
  ParameterDefinition* next() {return reinterpret_cast<ParameterDefinition*>((char*)(this)+Size() );}
};

struct ParameterRecord: public RecordHeader
{
  int32_t paramCount;
  ParameterDefinition* parameters() {return reinterpret_cast<ParameterDefinition*>(this+1);}
};

struct AttributeString
{
  int32_t strLen;
  char * str() {return (char*)(this)+sizeof(strLen);}
  int Size(){return strLen+sizeof(strLen);}
};

struct AttributeDefinition
{
  int32_t cookie;
  AttributeString attributeName;
  int32_t attributeLen() { return *reinterpret_cast<int32_t*>( (uint8_t*)(this)+sizeof(cookie)+attributeName.Size()); }
  char * attribute() {return reinterpret_cast<char*>(this)+sizeof(cookie)+attributeName.Size()+sizeof(int32_t);}
  AttributeDefinition* next() {return reinterpret_cast<AttributeDefinition*>(attribute() + attributeLen());}
}; 

struct AttributeRecord: public RecordHeader
{
  int32_t attribCount;
  AttributeDefinition* attributes() {return reinterpret_cast<AttributeDefinition*>(this+1);};
};

#pragma pack()

} // namespace packet
} // namespace simrad

#endif
