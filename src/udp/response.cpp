#include <simrad_ek80/udp/response.h>
#include <simrad_ek80/utilities.h>
#include <iostream>

namespace simrad
{

Response::Response(const std::string& method, const std::string& response, unsigned int rid)
  :method_(method), rid_(rid)
{
  //std::cerr << "Response: " << response << std::endl;
  std::istringstream rs(response);
  read_xml(rs,response_);
  if(response_.get<unsigned int>("response.clientInfo.rid") == rid)
    return;
  throw (Exception("Invalid Response"));
}

std::string Response::getArgument(const std::string& argument) const
{
  return response_.get("response."+method_+"Response."+replace(argument, '/', '.'),"");
}

Response::ArgumentList Response::getArgumentList(const std::string& argument) const
{
  ArgumentList ret;
  boost::property_tree::ptree::const_assoc_iterator n = response_.find("response");
  if(n != response_.not_found())
  {
    boost::property_tree::ptree::const_assoc_iterator mr = n->second.find(method_+"Response");
    if(mr != n->second.not_found())
    {
      boost::property_tree::ptree::const_iterator part = n->second.to_iterator(mr);
      while(part->second.begin() != part->second.end())
      {
        part = part->second.begin();
        ret.push_back(ArgumentPair(part->first, part->second.data()));
      }
    }
  }
  return ret;
}

Response::operator std::string() const
{
  std::stringstream ret;
  write_xml(ret,response_);
  return ret.str();
}

int Response::getErrorCode() const
{
  return response_.get("response.fault.detail.errorcode",-999);
}

std::string Response::getErrorMessage() const
{
  return response_.get("response.fault.detail.message",std::string("Error message not found in response!"));
}


} // namespace simrad
