#include <iostream>
#include <vector>
#include <simrad_ek80/file/raw_file.h>
#include <simrad_ek80/file/ek80.h>
#include <simrad_ek80/utilities.h>

int main(int argc, char **argv)
{
  if(argc < 2)
  {
    std::cerr << "usage: extract_nmea in1.raw [in2.raw ...]" << std::endl;
    return 1;
  }

  for(int i = 1; i < argc; i++)
  {
    std::cerr << argv[i] << std::endl;
    simrad::RawFile file(argv[i]);

    while(file.bytesLeft()>0)
    {
      std::vector<char> packet = file.readPacket();
      auto dh = file.asHeader(packet);
      switch(dh->DatagramType)
      {
        case simrad::file::NMEAType:
        {
          auto nmea = file.asType<simrad::file::NMEADatagram>(packet);
          std::cout << std::setprecision(13) << std::chrono::duration_cast<std::chrono::milliseconds>(simrad::fromSimradTime(nmea->header.DateTime).time_since_epoch()).count()/1000.0 << "," <<  &nmea->nmea << std::endl;
          break;
        }
        default:
        {
        }
      }
    }
  }

  return 0;
}
