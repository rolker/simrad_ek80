#include <simrad_ek80/sample.h>
#include <simrad_ek80/channel.h>
#include <cmath>
#include <iostream>

namespace simrad
{

SampleSubscription::SampleSubscription(std::shared_ptr<Channel> channel, std::string sample_data_type)
  :Subscription("SampleData", channel->getName()), sample_data_type_(sample_data_type), channel_(channel)
{
}

std::shared_ptr<SampleSubscription> SampleSubscription::subscribe(std::shared_ptr<Channel> channel, float range, std::string sample_data_type)
{
  auto sub = std::make_shared<SampleSubscription>(channel, sample_data_type);
  sub->setParameter("SampleDataType", sample_data_type);
  sub->setParameter("Range", std::to_string(range));
  channel->subscribe(sub);
  return sub;
}

void SampleSubscription::addData(std::shared_ptr<std::vector<unsigned char> > &data)
{
  //std::cout << "SampleSubscription::addData: " << data->size() << "bytes" << std::endl;
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

    double interval = channel_->getParameter("SampleInterval")->get<double>(std::nan(""));
    double sspeed = channel_->getParameter("SoundVelocity")->get<double>(std::nan(""));
    double depth = 0.0;
    
    SampleSet::Ptr ss(new SampleSet);
    ss->time = fromSimradTime(d->time);
    ss->frequency = channel_->getParameter("Frequency")->get<double>(0.0);
    ss->z1 = -depth;
    ss->sampleInterval = interval;
    ss->soundSpeed = sspeed;
    ss->dataType = sample_data_type_;
    ss->beamWidthX = channel_->getParameter("BeamWidthAthwartship")->get<double>(0.0);
    ss->beamWidthY = channel_->getParameter("BeamWidthAlongship")->get<double>(0.0);
    ss->directionX = channel_->getParameter("DirectionX")->get<double>(0.0);
    ss->directionY = channel_->getParameter("DirectionY")->get<double>(0.0);
    
    int count = (data->size()-sizeof(uint64_t))/sizeof(uint16_t);
    
    for(int i = 0; i < count; ++i)
    {
        ss->samples.push_back(d->data[i]);
    }
    callCallbacks(ss);
  }
}

} // namespace simrad
