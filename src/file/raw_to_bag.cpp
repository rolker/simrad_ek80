#include <iostream>
#include <vector>
#include <simrad_ek80/file/raw_file.h>
#include <simrad_ek80/file/ek80.h>
#include <acoustic_msgs/RawSonarImage.h>
#include <rosbag/bag.h>

int main(int argc, char **argv)
{
  if(argc < 3)
  {
    std::cerr << "usage: raw_to_bag in.raw out.bag" << std::endl;
    return 1;
  }

  simrad::RawFile file(argv[1]);
  rosbag::Bag bag;
  bag.open(argv[2], rosbag::bagmode::Write);

  std::cerr << argv[1] << " byte count: " << file.size() << std::endl;

  simrad::file::EK80 ek;

  while(file.bytesLeft()>0)
  {
    std::vector<char> packet = file.readPacket();
    auto dh = file.asHeader(packet);
    switch(dh->DatagramType)
    {
      case simrad::file::XMLType:
      {
        auto xml = file.asType<simrad::file::XMLDatagram>(packet);
        std::cerr << &xml->xml << std::endl;
        ek.processXML(&xml->xml);
        break;
      }
      case simrad::file::NMEAType:
      {
        auto nmea = file.asType<simrad::file::NMEADatagram>(packet);
        //std::cerr << &nmea->nmea << std::endl;
        break;
      }
      case simrad::file::SampleType3:
      {
        auto sample = file.asType<simrad::file::SampleDatagram3>(packet);
        auto ping = ek.samplesToPowers(*sample);
        acoustic_msgs::RawSonarImage rsi;
        rsi.image.beam_count = 1;
        rsi.image.dtype = acoustic_msgs::SonarImageData::DTYPE_FLOAT32;
        rsi.image.data.resize(ping.samples.size()*4);
        std::memcpy(&rsi.image.data.front(), reinterpret_cast<const uint8_t*>(&ping.samples.front()), rsi.image.data.size());
        rsi.rx_angles.push_back(0.0);
        rsi.tx_angles.push_back(0.0);
        rsi.sample0 = sample->Offset;
        rsi.samples_per_beam = sample->Count;
        rsi.ping_info.sound_speed = ping.sound_speed;
        rsi.sample_rate = 1.0/ping.sample_interval;
        rsi.ping_info.rx_beamwidths.push_back(ping.beamwidth_athwartship*M_PI/180.0);
        rsi.ping_info.tx_beamwidths.push_back(ping.beamwidth_alongship*M_PI/180.0);
        rsi.header.stamp = ros::Time(std::chrono::duration_cast<std::chrono::nanoseconds>(ping.timestamp.time_since_epoch()).count()/1000000000.0);
        rsi.header.frame_id = "ek80";
        bag.write("sonar_image", rsi.header.stamp, rsi);
        // for(int i = 0; i < ping.samples.size() && i < 250; i++)
        //   std::cerr << ping.samples[i] << " ";
        // std::cerr << std::endl;
        // exit(0);
        break;
      }
      case simrad::file::MRUType:
      {
        auto mru = file.asType<simrad::file::MRUDatagram>(packet);
        //std::cerr << "Heading: " << mru->Heading << ", Pitch: " << mru->Pitch << ", Roll: " << mru->Roll << ", Heave: " << mru->Heave << std::endl;
        ek.processMRU(*mru);
        break;
      }
      case simrad::file::FilterType:
      {
        auto filter = file.asType<simrad::file::FilterDatagram>(packet);
        std::cerr << "Filter type: " << int(filter->FilterType) << " number of coefficients: " << filter->NoOfCoefficients << std::endl;
        break;
      }
      default:
      {
        std::cerr << "type: 0x" << std::hex << dh->DatagramType << std::dec << std::endl;
        std::cerr << "packet size: " << packet.size() << std::endl;
      }
    }
  }

  bag.close();
  return 0;
}
