#ifndef SIMRAD_EK80_RESPONSE_H
#define SIMRAD_EK80_RESPONSE_H

#include <boost/property_tree/xml_parser.hpp>
#include <vector>
#include <memory>

namespace simrad
{

class Response
{
public:
  using ArgumentPair = std::pair<std::string, std::string>;
  using ArgumentList = std::vector<ArgumentPair>;

  Response(const std::string& method, const std::string& response, unsigned int rid);

  std::string getArgument(const std::string& argument) const;
                
  ArgumentList getArgumentList(const std::string& argument) const;
                
  operator std::string() const;
                
  int getErrorCode() const;
                
  std::string getErrorMessage() const;
private:
  boost::property_tree::ptree response_;
  std::string method_;
  unsigned int rid_;
};

} // namespace simrad

#endif
