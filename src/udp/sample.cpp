#include <simrad_ek80/udp/sample.h>
#include <simrad_ek80/udp/channel.h>
#include <cmath>
#include <iostream>

namespace simrad
{

SampleSubscription::SampleSubscription(const Channel& channel, int range, std::string sample_data_type, int start_range)
  :Subscription("SampleData", channel.name()), range_(range), start_range_(start_range), sample_data_type_(sample_data_type), channel_(channel)
{
  setParameter("SampleDataType", sample_data_type);
  setParameter("Range", std::to_string(range));
  setParameter("RangeStart", std::to_string(start_range));
}

void SampleSubscription::addData(std::shared_ptr<std::vector<unsigned char> > &data)
{
  #pragma pack(1)
  struct SampleData
  {
      uint64_t time;
      int16_t data[1];
  };
  #pragma pack()
    
  if(data->size())
  {
    SampleData * d = reinterpret_cast<SampleData *>(&data->front());

    auto interval_parameter = channel_.getParameter("SampleInterval");
    if(!interval_parameter)
      return;
    double interval = interval_parameter->get<double>(std::nan(""));

    auto sound_speed_parameter = channel_.getParameter("SoundVelocity");
    if(!sound_speed_parameter)
      return;
    double sspeed = sound_speed_parameter->get<double>(std::nan(""));
    double depth = 0.0;
    
    SampleSet::Ptr ss(new SampleSet);
    ss->time = fromSimradTime(d->time);
    ss->frequency = channel_.getParameter("Frequency")->get<double>(0.0);
    ss->range = range_;
    ss->start_range = start_range_;
    ss->z1 = -depth;
    ss->sampleInterval = interval;
    ss->soundSpeed = sspeed;
    ss->dataType = sample_data_type_;

    auto beamwidthx_param = channel_.getParameter("BeamWidthAthwartship");
    if (beamwidthx_param)
      ss->beamWidthX = beamwidthx_param->get<double>(0.0);

    auto beamwidthy_param = channel_.getParameter("BeamWidthAlongship");
    if (beamwidthy_param)
      ss->beamWidthY = beamwidthy_param->get<double>(0.0);

    auto direction_x_param = channel_.getParameter("DirectionX");
    if(direction_x_param)
      ss->directionX = direction_x_param->get<double>(0.0);

    auto direction_y_param = channel_.getParameter("DirectionY");
    if(direction_y_param)
      ss->directionY = direction_y_param->get<double>(0.0);
    
    int count = (data->size()-sizeof(uint64_t))/sizeof(uint16_t);
    
    for(int i = 0; i < count; ++i)
    {
        ss->samples.push_back(d->data[i]);
    }
    callCallbacks(ss);
  }
}


EchogramSubscription::EchogramSubscription(const Channel& channel, int range, int start_range, float bin_size, std::string tvg_type)
  :Subscription("Echogram", channel.name()), channel_(channel)
{
  int pixel_count = range/bin_size;
  setParameter("PixelCount", std::to_string(pixel_count));
  setParameter("Range", std::to_string(range));
  setParameter("RangeStart", std::to_string(start_range));
  setParameter("TVGType", tvg_type);
}

void EchogramSubscription::addData(std::shared_ptr<std::vector<unsigned char> > &data)
{
  #pragma pack(1)
  struct EchogramHeader
  {
    uint64_t time;
    double range;
    double rangeStart;
  };

  struct EchogramData: public EchogramHeader
  {
    int16_t data[1];
  };
  #pragma pack()
    
  if(data->size())
  {
    EchogramData * d = reinterpret_cast<EchogramData *>(&data->front());

    int sample_count = (data->size()-sizeof(EchogramHeader))/sizeof(int16_t); 


    auto sound_speed_parameter = channel_.getParameter("SoundVelocity");
    if(!sound_speed_parameter)
      return;
    double sspeed = sound_speed_parameter->get<double>(std::nan(""));
    double depth = 0.0;

    double interval_meters = d->range/double(sample_count);
    double interval_seconds = 2.0*interval_meters/sspeed;


    SampleSet::Ptr ss(new SampleSet);
    ss->time = fromSimradTime(d->time);
    ss->frequency = channel_.getParameter("Frequency")->get<double>(0.0);
    ss->range = d->range;
    ss->start_range = d->rangeStart;
    ss->z1 = -depth;
    ss->sampleInterval = interval_seconds;
    ss->soundSpeed = sspeed;
    ss->dataType = "echogram";

    auto beamwidthx_param = channel_.getParameter("BeamWidthAthwartship");
    if (beamwidthx_param)
      ss->beamWidthX = beamwidthx_param->get<double>(0.0);

    auto beamwidthy_param = channel_.getParameter("BeamWidthAlongship");
    if (beamwidthy_param)
      ss->beamWidthY = beamwidthy_param->get<double>(0.0);

    auto direction_x_param = channel_.getParameter("DirectionX");
    if(direction_x_param)
      ss->directionX = direction_x_param->get<double>(0.0);

    auto direction_y_param = channel_.getParameter("DirectionY");
    if(direction_y_param)
      ss->directionY = direction_y_param->get<double>(0.0);
    
    int count = (data->size()-sizeof(uint64_t))/sizeof(uint16_t);
    
    for(int i = 0; i < count; ++i)
    {
        ss->samples.push_back(d->data[i]);
    }
    callCallbacks(ss);
  }
}


} // namespace simrad
