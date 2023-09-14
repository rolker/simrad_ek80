#include <simrad_ek80/udp/request.h>
#include <iostream>

namespace simrad
{

Request::Request(Connection::Ptr& connection, const std::string& target, const std::string& method)
  :method_(method), connection_(connection)
{
  request_.put("request.clientInfo.cid", connection->getID());
  rid_ = connection->getRID();
  request_.put("request.clientInfo.rid", rid_);

  request_.put("request.type","invokeMethod");
  request_.put("request.targetComponent",target);
}

void Request::addArgument(const std::string& argument, const std::string& value)
{
  request_.put("request.method."+method_+"."+argument, value);
}

Response Request::getResponse()
{
  while(true)
  {
    try
    {
      std::stringstream r;
      write_xml(r,request_);
      return Response(method_, connection_->sendRequest(r.str(), rid_), rid_);
    }
    catch (Exception e)
    {
      std::cerr << "simrad::Request::getResponse\t" << e.what() << std::endl;
    }
  }
}

} // namespace simrad
