#include <simrad_ek80/utilities.h>

#include <sstream>
#include <vector>
#include <net/if.h>
#include <arpa/inet.h>
#include <algorithm>

//#include <iostream>

namespace simrad
{

TimePoint fromSimradTime(uint64_t t)
{
  // Simrad uses a 64-bit integer value stating the number of
  // 100 nanosecond intervals since January 1, 1601.
  // This is the internal "filetime" used by the Windows
  // operating systems.
  //std::cerr << "simrad time to unix: " << t << " -> " << std::chrono::duration_cast<std::chrono::seconds>(TimePoint(std::chrono::nanoseconds((t-0x019db1ded53e8000ULL)*100)).time_since_epoch()).count() << std::endl;
  if(t < 0x019db1ded53e8000ULL)
    return TimePoint();
  return TimePoint(std::chrono::nanoseconds((t-0x019db1ded53e8000ULL)*100));
}

Exception::Exception(const std::string& message):message_(message)
{
}

Exception::~Exception() throw()
{
}

const char* Exception::what() const throw()
{
  return message_.c_str();  
}

AccessDeniedException::AccessDeniedException(const std::string& message):Exception(message)
{
}

InvalidParameterException::InvalidParameterException(const std::string& message):Exception(message)
{
}

std::string mapToString(const std::map<std::string,std::string>& map, std::string joiner)
{
  std::string ret;
  for(auto i = map.begin(); i != map.end(); ++i)
  {
    if(!ret.empty())
      ret += joiner;
    ret += i->first + ":" + i->second;
  }
  return ret;  
}

std::map<std::string,std::string> stringToMap(const std::string & s)
{
  std::map<std::string,std::string> ret;
  int i = 0;
  int key_start = 0;
  int key_end = -1;
  int value_start = -1;
  int bracket_level = 0;
  while(i < s.size())
  {
    if(key_end < 0) // in key
    {
      if (i == key_start && s[i] == ' ')
        key_start += 1;
      if (s[i] == ':')
      {
        key_end = i;
        value_start = i+1;
      }
    }
    else // in value
    {
      if (s[i] == '{')
        bracket_level += 1;
      if (s[i] == '}')
        bracket_level -= 1;
      if (bracket_level == 0 && s[i] == ',')
      {
        ret[s.substr(key_start,key_end-key_start)] = s.substr(value_start, i-value_start);
        key_start = i+1;
        key_end = -1;
      }
    }
    i += 1;
    if (i == s.size())
    {
      // save last key/value
      if(key_end > 0)
      {
        int value_end = i;
        while(value_end > value_start && s[value_end-1] == ' ')
          value_end -= 1;
        ret[s.substr(key_start, key_end-key_start)] =  s.substr(value_start, value_end-value_start);
      }
    }
  }
  return ret;
}

bool validInterface(const ifaddrs* i)
{
  return i && i->ifa_addr && i->ifa_addr->sa_family == AF_INET && (i->ifa_flags & IFF_UP) > 0 && (i->ifa_flags & IFF_LOOPBACK) == 0;
}

std::vector<uint32_t> getLocalAddresses(bool broadcast)
{
  std::vector<uint32_t> ret;
  ifaddrs *addr_list;
  if (!getifaddrs(&addr_list))
  {
    for (ifaddrs * addr = addr_list; addr; addr = addr->ifa_next)
      if(validInterface(addr))
        if(broadcast)
          ret.push_back(((sockaddr_in *)(addr->ifa_ifu.ifu_broadaddr))->sin_addr.s_addr);
        else
          ret.push_back(((sockaddr_in *)(addr->ifa_addr))->sin_addr.s_addr);
    freeifaddrs(addr_list);
  }
  return ret;
}

std::string ipAddressToString(uint32_t a)
{
    std::stringstream ret;
    const uint8_t *bytes = reinterpret_cast<const uint8_t*>(&a);
    ret << int(bytes[0]) << "." << int(bytes[1]) << "." << int(bytes[2]) << "." << int(bytes[3]);
    return ret.str();
}

uint32_t ipAddressFromString(const std::string &a)
{
    return inet_addr(a.c_str());
}

std::string & trim(std::string & str)
{
   return ltrim(rtrim(str));
}

std::string & ltrim(std::string & str)
{
  auto it2 =  std::find_if( str.begin() , str.end() , [](char ch){ return !std::isspace<char>(ch , std::locale::classic() ) ; } );
  str.erase( str.begin() , it2);
  return str;   
}

std::string & rtrim(std::string & str)
{
  auto it1 =  std::find_if( str.rbegin() , str.rend() , [](char ch){ return !std::isspace<char>(ch , std::locale::classic() ) ; } );
  str.erase( it1.base() , str.end() );
  return str;   
}

std::string trim_copy(std::string const & str)
{
   auto s = str;
   return ltrim(rtrim(s));
}

std::vector<std::string> split(std::string input, std::string delimiter)
{
  std::vector<std::string> ret;
  std::size_t pos = 0;
  while ((pos = input.find(delimiter)) != std::string::npos)
  {
    ret.push_back(input.substr(0,pos));
    input.erase(0, pos + delimiter.length());
  }
  ret.push_back(input);
  return ret;
}

std::string replace(const std::string& string, char character, char replacement)
{
  std::string ret;
  for(auto c: string)
    if( c == character)
      ret.append(1, replacement);
    else
      ret.append(1, c);
  return ret;
}

std::string toLower(const std::string& string)
{
  std::string lower = string;
  std::transform(lower.begin(), lower.end(), lower.begin(),
    [](unsigned char c){ return std::tolower(c); });
  return lower;
}

std::string channelNameToTopicName(const std::string& name)
{
  auto parts = simrad::split(name, " ");
  auto topic = simrad::replace(parts.back(), '-', '_');
  return toLower(topic);
}

} // namespace simrad
