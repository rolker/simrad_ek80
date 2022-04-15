#ifndef SIMRAD_EK80_SIMRAD_H
#define SIMRAD_EK80_SIMRAD_H

#include <exception>
#include <string>
#include <map>
#include <ifaddrs.h>
#include <vector>
#include <chrono>

namespace simrad
{

using Clock = std::chrono::system_clock;
using TimePoint = std::chrono::time_point<Clock>;
TimePoint fromSimradTime(uint64_t t);

class Exception: public std::exception
{
public:
  Exception(const std::string& message);
  ~Exception() throw();
  const char* what() const throw();
protected:
  std::string message_;
};

class AccessDeniedException: public Exception
{
public:
  AccessDeniedException(const std::string& message);
};

class InvalidParameterException: public Exception
{
public:
  InvalidParameterException(const std::string& message);
};


std::string mapToString(const std::map<std::string,std::string>& map, std::string joiner=",");
std::map<std::string,std::string> stringToMap(const std::string & s);

inline float rawToDB(int16_t r){return r*0.01175898420562426544;}

bool validInterface(const ifaddrs* i);
std::vector<uint32_t> getLocalAddresses(bool broadcast=false);
std::string ipAddressToString(uint32_t a);
uint32_t ipAddressFromString(const std::string &a);

std::string & trim(std::string & str);
std::string & ltrim(std::string & str);
std::string & rtrim(std::string & str);
std::string trim_copy(std::string const & str);

namespace swap_private
{
  template <int S> struct SwapSize
  {
    int dummy;
    SwapSize(){}
  };

  template <typename T> T swap(T const &value, SwapSize<1> const &s)
  {
    return value;
  }

  template <typename T> T swap(T const &value, SwapSize<2> const &s)
  {
    return((value << 8)&0xff00)|((value>>8)&0x00ff);
  }

  template <typename T> T swap(T const &value, SwapSize<4> const &s)
  {
    return  ((value << 24)&0xff000000)|
            ((value << 8)&0x00ff0000)|
            ((value >> 8)&0x0000ff00)|
            ((value>>24)&0x000000ff);
  }

  template <typename T> T swap(T const &value, SwapSize<8> const &s)
  {
    return  ((value&0x000000ff) << 56)|
            ((value&0x0000ff00) << 40)|
            ((value&0x00ff0000) << 24)|
            ((value&0xff000000) << 8)|
            ((value >> 8)&0xff000000)|
            ((value >> 24)&0x00ff0000)|
            ((value >> 40)&0x0000ff00)|
            ((value >> 56)&0x000000ff);
  }
}

template<typename T> T swap(const T& value)
{
  return swap_private::swap(value, swap_private::SwapSize<sizeof(T)>());
}

template<typename T> T swap(const T& value, bool s)
{
  if(s)
    return swap(value);
  return value;
}

std::string replace(const std::string& string, char character, char replacement);

} // namespace simrad

#endif
