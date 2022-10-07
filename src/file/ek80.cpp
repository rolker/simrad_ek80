#include <simrad_ek80/file/ek80.h>
#include <simrad_ek80/utilities.h>
#include <boost/property_tree/xml_parser.hpp>
#include <iostream>
#include <complex>

namespace simrad
{
namespace file
{

void EK80::processXML(std::string xml)
{
  std::istringstream rs(xml);

  boost::property_tree::ptree tree;
  read_xml(rs, tree);

  for(auto element: tree)
  {
    if(element.first == "Configuration")
    {
      auto transceivers = element.second.get_child_optional("Transceivers");
      if(transceivers.is_initialized())
      {
        for(auto t: *transceivers)
        {
          if(t.first == "Transceiver")
          {
            auto transceiver_name = t.second.get<std::string>("<xmlattr>.TransceiverName");
            transceivers_[transceiver_name].configure(t.second);
            auto channels = t.second.get_child_optional("Channels");
            if(channels.is_initialized())
            {
              for(auto c: *channels)
              {
                if(c.first == "Channel")
                {
                  auto channel_name = c.second.get<std::string>("<xmlattr>.ChannelID");
                  channels_[channel_name].configure(c.second);
                  channels_[channel_name].setTransceiver(transceiver_name);
                  for(auto channel_child: c.second)
                  {
                    if(channel_child.first == "Transducer")
                    {
                      auto transducer_name = channel_child.second.get<std::string>("<xmlattr>.TransducerName");
                      transducers_[transducer_name].configure(channel_child.second);
                    }
                  }
                }
              }
            }
          }
        }
      }

      auto transducers = element.second.get_child_optional("Transducers");
      if(transducers.is_initialized())
      {
        for(auto t: *transducers)
        {
          if(t.first == "Transducer")
          {
            auto transducer_name = t.second.get<std::string>("<xmlattr>.TransducerName");
            transducers_[transducer_name].configure(t.second);
          }
        }
      }

    }
    else if(element.first == "InitialParameter")
    {
      auto channels = element.second.get_child_optional("Channels");
      if(channels.is_initialized())
      {
        for(auto c: *channels)
        {
          if(c.first == "Channel")
          {
            auto channel_name = c.second.get<std::string>("<xmlattr>.ChannelID");
            channels_[channel_name].updateParameters(c.second);
          }
        }
      }

    }
    else if(element.first == "Parameter")
    {
      for(auto c: element.second)
      {
        if(c.first == "Channel")
        {
          auto channel_name = c.second.get<std::string>("<xmlattr>.ChannelID");
          channels_[channel_name].updateParameters(c.second);
        }
      }
    }
    else if(element.first == "Parameter")
    {

    }
    else
    {
      std::cerr << "skipping " << element.first << std::endl;
    }
  }
}

EK80::Ping EK80::samplesToPowers(const SampleDatagram3& samples) const
{
  //std::cerr << samples.Count <<  " samples from channel: " << samples.ChannelID << " data type: 0x" << std::hex << samples.Datatype << std::dec << " offset: " << samples.Offset << std::endl;

  Ping ret;
  ret.timestamp = fromSimradTime(samples.DgHeader.DateTime);
  auto tt = std::chrono::system_clock::to_time_t(ret.timestamp);
  //std::cerr << std::put_time(std::gmtime(&tt), "%FT%T") << std::endl;

  if((samples.Datatype&0x0f) == 0x08) // Complex float32
  {
    int values_per_sample = (samples.Datatype >> 8)&0x7;
    //std::cerr << values_per_sample << " values per sample" << std::endl;
    const std::complex<float>* sample_data = reinterpret_cast<const std::complex<float>*>(&samples.Samples);

    std::string channel_id = (const char *)(samples.ChannelID);
    auto channel_iterator = channels_.find(channel_id);
    if(channel_iterator != channels_.end())
    {
      auto transceiver = transceivers_.find(channel_iterator->second.transceiver());
      auto transducer = transducers_.find(channel_iterator->second.transducer());
      double transceiver_impedance = transceiver->second.impedance();
      double transducer_impedance = transducer->second.impedance(channel_iterator->second.frequency());

      double zoffset = transducer->second.offset_z() + heave_;
      ret.sound_speed = channel_iterator->second.sound_speed();
      ret.sample_interval = channel_iterator->second.sample_interval();
      ret.beamwidth_alongship = transducer->second.beamwidth_alongship();
      ret.beamwidth_athwartship = transducer->second.beamwidth_athwartship();

      for(int i = 0; i < samples.Count; i++)
      {
        std::complex<float> sum = 0.0;
        for(int j = 0; j < values_per_sample; j++)
        {
          sum += sample_data[i*values_per_sample+j];
        }
        //sum /= values_per_sample;
        double range = (samples.Offset+i)*ret.sample_interval*ret.sound_speed;
        double power = values_per_sample*pow(abs(sum)/(2.0*sqrt(2.0)),2.0)*pow((transceiver_impedance+transducer_impedance)/transceiver_impedance,2.0)/transducer_impedance;
        //double db = 10.0*log(power/channel_iterator->second.transmit_power());
        double db = 10.0*log(power);
        double tvg = 20*log(range);
        if(range < 1.0)
          tvg = 0.0;
        double ac = 2*0.01*range;
        ret.samples.push_back(db+tvg+ac);
      }
    }
    else
      std::cerr << "channel not found: " << samples.ChannelID << std::endl;
  }
  else
    std::cerr << "not complex float32" << std::endl;
  return ret;
}

void EK80::processMRU(const MRUDatagram& mru)
{
  heading_ = mru.Heading;
  pitch_ = mru.Pitch;
  roll_ = mru.Roll;
  heave_ = mru.Heave;
}

const float& EK80::heading() const
{
  return heading_;
}

const float& EK80::pitch() const
{
  return pitch_;
}

const float& EK80::roll() const
{
  return roll_;
}

const float& EK80::heave() const
{
  return heave_;
}

} // namepsace file
} // namespace simrad
