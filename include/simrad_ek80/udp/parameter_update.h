#ifndef SIMRAD_EK80_PARAMETER_UPDATE_H
#define SIMRAD_EK80_PARAMETER_UPDATE_H

#include <simrad_ek80/utilities.h>
#include <vector>

namespace simrad
{

struct ParameterUpdate
{
  int id;
  std::vector<char> value;
  TimePoint time;

  template<typename T> operator T() const;
};

template<typename T> inline ParameterUpdate::operator T() const
{
  return *reinterpret_cast<T const*>(&value.front());
}

template<> inline ParameterUpdate::operator std::string() const
{
  return std::string(value.begin(),value.end());
}

template<> inline ParameterUpdate::operator std::vector<std::string>() const
{
  std::vector<std::string> ret;
  char const *cursor = &value.front();
  while(cursor != &(*value.end()))
  {
    ret.push_back(cursor);
    cursor = &cursor[ret.back().size()+1];
  }
  return ret;
}

template<> inline ParameterUpdate::operator std::vector<double>() const
{
  double const *ret = reinterpret_cast<double const*>(&value.front());
  return std::vector<double>(ret,&ret[value.size()/sizeof(double)]);
}

} // namespace simrad

#endif
