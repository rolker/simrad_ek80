#include <simrad_ek80/file/raw_file.h>
#include <simrad_ek80/utilities.h>

namespace simrad
{

RawFile::RawFile(std::string filename)
  :infile_(filename, std::ios::in|std::ios::binary|std::ios::ate)
{
  if(!infile_.is_open())
    throw Exception("Can't open "+filename);
  file_size_ = infile_.tellg();
  infile_.seekg(0);
}

uint32_t RawFile::size() const
{
  return file_size_;
}

uint32_t RawFile::bytesLeft()
{
  auto p = infile_.tellg();
  return file_size_-p;
}

std::vector<char> RawFile::readPacket()
{
  uint32_t packet_size;
  infile_.read(reinterpret_cast<char*>(&packet_size),4);
  std::vector<char> ret(packet_size);
  infile_.read(ret.data(), packet_size);
  uint32_t packet_size_check;
  infile_.read(reinterpret_cast<char*>(&packet_size_check),4);
  if(packet_size != packet_size_check)
    throw Exception("Missmatch packet sizes: "+std::to_string(packet_size)+" vs "+std::to_string(packet_size_check));
  return ret;
}

bool RawFile::eof() const
{
  return infile_.eof();
}

const file::DatagramHeader* RawFile::asHeader(const std::vector<char>& data) const
{
  if(data.size() < sizeof(file::DatagramHeader))
    return nullptr;
  return reinterpret_cast<const simrad::file::DatagramHeader*>(data.data());
}

} // namespace simrad
