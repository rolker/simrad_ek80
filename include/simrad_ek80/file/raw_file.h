#ifndef SIMRAD_EK80_FILE_RAW_FILE_H
#define SIMRAD_EK80_FILE_RAW_FILE_H

#include <simrad_ek80/file/format.h>
#include <string>
#include <fstream>
#include <vector>

namespace simrad
{

class RawFile
{
public:
  RawFile(std::string filename);
  uint32_t size() const;
  std::vector<char> readPacket();
  bool eof() const;
  uint32_t bytesLeft();

  const file::DatagramHeader* asHeader(const std::vector<char>& data) const;

  template<typename T> const T* asType(const std::vector<char>& data) const
  {
    auto header = asHeader(data);
    if(header && header->DatagramType == T::type)
      return reinterpret_cast<const T*>(data.data());
    return nullptr;
  }

private:
  std::ifstream infile_;
  std::streampos file_size_;
};

} // namespace simrad

#endif
