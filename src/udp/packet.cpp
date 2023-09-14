#include <simrad_ek80/udp/packet.h>
#include <cstring>
#include <sstream>
#include <iostream>
#include <boost/property_tree/xml_parser.hpp>

namespace simrad
{
namespace packet
{

Base::Base(std::string type)
{
  memset(Header,0,4);
  strcpy(Header,type.substr(0,3).c_str());
}

bool Base::isHeader(std::string h) const
{
  return h == Header;
}

Base::operator char const*() const
{
  return (char const *)this;
}

RequestServerInfo::RequestServerInfo():Base("RSI")
{

}

RequestServerInfo::operator bool() const
{
  return isHeader("RSI");
}

ServerInfo2::operator bool() const
{
  return isHeader("SI2");
}

ClientInfo::ClientInfo(std::string type):Base(type)
{
  memset(Info,0,1024);
}

void ClientInfo::setInfo(std::string const &info)
{
  if(info.size() > 1023)
    throw(Exception("Data too long to fit in packet"));
  memset(Info,0,1024);
  strcpy(Info,info.c_str());
}

Connection::Connection(std::string type):ClientInfo(type)
{

}

void Connection::setNamePwd(std::string username, std::string password)
{
  std::stringstream ss;
  ss << "Name:" << username << ";Password:" << password;
  //if(!password.empty())
  //  ss << " " << password;
  setInfo(ss.str());
}

ConnectRequest::ConnectRequest():Connection("CON")
{

}

DisconnectRequest::DisconnectRequest():Connection("DIS")
{

}

AliveReport::AliveReport():ClientInfo("ALI")
{

}
  
AliveReport::operator bool() const
{
  return isHeader("ALI");
}

void AliveReport::setInfo(std::string clientID, int seqNo)
{
  std::map<std::string, std::string> params;
  params["ClientID"] = clientID;
  params["SeqNo"] = std::to_string(seqNo);
  ClientInfo::setInfo(mapToString(params));
}

Response::operator bool() const
{
  return isHeader("RES");
}

ConnectResponse::operator bool() const
{
  return isHeader("RES") && std::string("CON") == Request;
}

std::map<std::string, std::string> ConnectResponse::getResult() const
{
  //std::cout << "MsgResponse: " << MsgResponse << std::endl;
  std::map<std::string, std::string> ret = stringToMap(MsgResponse);
  // for (auto kv: ret)
  //   std::cout << kv.first << "|" << kv.second << "|" << std::endl;

  if (ret.find("ResultCode") == ret.end())
    throw(Exception("ResultCode not found in "+std::string(MsgResponse)));
      
  if (ret["ResultCode"] != "S_OK")
  {
    std::string reason("Unknown error:"+std::string(MsgResponse));
    if(ret.find("ResultInfo") != ret.end())
      reason = ret["ResultInfo"];
          
    if (ret["ResultCode"] == "E_ACCESSDENIED")
      throw(AccessDeniedException(reason));
    throw(Exception(reason));
  }
      
  if(ret.find("Parameters") == ret.end())
    throw(Exception("Unexpected parameter in OK response"));

  auto params = ret["Parameters"];
  if (!params.empty() && params[0] == '{')
    params.erase(params.begin());
  if (!params.empty() && params.back() == '}')
    params.erase(params.size()-1);
  //std::cout << "params: " << params << std::endl;

  return stringToMap(params);
}

RequestResponse::operator bool() const
{
  return isHeader("RES") && std::string("REQ") == Request;
}

unsigned int RequestResponse::getRequestID() const
{
  std::istringstream rs(MsgResponse);
  boost::property_tree::ptree response;
  read_xml(rs, response);
  return response.get<unsigned int>("response.clientInfo.rid");
}

Retransmit::operator bool() const
{
  return isHeader("RTR");
}

Request::Request():Base("REQ")
{
  memset(MsgControl,0,1422);
}

ProcessedData::operator bool() const
{
  return isHeader("PRD");
}

} // namespace packet
} // namespace simrad
