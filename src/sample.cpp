#include <simrad_ek80/sample.h>
#include <cmath>
#include <iostream>

namespace simrad
{

SampleSubscription::SampleSubscription(std::string channel, const ParameterGroup::Map& ping_parameters, std::string sample_data_type)
  :Subscription("SampleData", channel, ping_parameters), sample_data_type_(sample_data_type)
{
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
      
    double interval = (*ping_parameters_["channel"])["sampleInterval"]->get<double>(std::nan(""));
    double sspeed = (*ping_parameters_["channel"])["soundSpeed"]->get<double>(std::nan(""));
    double depth = 0.0;
    //if((*ping_parameters_["channel"])["transducerDepth"])
    //  depth = (*ping_parameters_["channel"])["transducerDepth"]->get<double>(depth);
      
    // double dirX = 0.0;
    // if((*pingParameters["channel"])["directionX"])
    //     dirX = (*pingParameters["channel"])["directionX"]->Get<double>(Nan<double>());
    
    // double dirY = 0.0;
    // if((*pingParameters["channel"])["directionY"])
    //     dirY = (*pingParameters["channel"])["directionY"]->Get<double>(Nan<double>());
    
    // Rotation<double> roll (-Angle<double,Radian>(dirX),Point<double>(0.0,1.0,0.0));
    // Rotation<double> pitch (Angle<double,Radian>(dirY),Point<double>(1.0,1.0,0.0));
    
    // Point<double> unitVec = pitch(roll(Point<double>(0.0,0.0,interval*sspeed)));
    
    SampleSet::Ptr ss(new SampleSet);
    ss->time = fromSimradTime(d->time);
    ss->frequency = (*ping_parameters_["channel"])["frequency"]->get<double>(0.0);
    ss->z1 = -depth;
    ss->sampleInterval = interval;
    ss->soundSpeed = sspeed;
    ss->dataType = sample_data_type_;
    ss->beamWidthX = (*ping_parameters_["channel"])["beamWidthX"]->get<double>(0.0);
    ss->beamWidthY = (*ping_parameters_["channel"])["beamWidthY"]->get<double>(0.0);
    ss->directionX = (*ping_parameters_["channel"])["directionX"]->get<double>(0.0);
    ss->directionY = (*ping_parameters_["channel"])["directionY"]->get<double>(0.0);
    
    int count = (data->size()-sizeof(uint64_t))/sizeof(uint16_t);
    
    for(int i = 0; i < count; ++i)
    {
        ss->samples.push_back(d->data[i]);
    }
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    for(auto callback: callbacks_)
      callback(ss);
  }
}

void SampleSubscription::addCallback(std::function<void(std::shared_ptr<SampleSet>)> callback)
{
  std::lock_guard<std::mutex> lock(callbacks_mutex_);
  callbacks_.push_back(callback);
}

} // namespace simrad
