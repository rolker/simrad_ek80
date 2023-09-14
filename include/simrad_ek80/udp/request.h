#ifndef SIMRAD_EK80_REQUEST_H
#define SIMRAD_EK80_REQUEST_H

#include <simrad_ek80/udp/connection.h>
#include <simrad_ek80/udp/response.h>
#include <boost/property_tree/xml_parser.hpp>

namespace simrad
{

class Request
{
public:
  Request(Connection::Ptr& connection, const std::string& target, const std::string& method);
                
  void addArgument(const std::string& argument, const std::string& value);
                
  Response getResponse();
                
private:
  boost::property_tree::ptree request_;
  std::string method_;
  Connection::Ptr connection_;
  unsigned int rid_;
};

} // namespace simrad

#endif
