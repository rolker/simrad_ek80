#ifndef SIMRAD_EK80_SAMPLE_H
#define SIMRAD_EK80_SAMPLE_H

#include <simrad_ek80/subscription.h>
#include <list>

namespace simrad
{

struct SampleSet
{
  TimePoint time;
  float frequency;
  float z1;
  float sampleInterval;
  float soundSpeed;
  float beamWidthX;
  float beamWidthY;
  float directionX;
  float directionY;
  std::string dataType;
  std::vector<int16_t> samples;
  typedef std::shared_ptr<SampleSet> Ptr;
};

class SampleSubscription: public Subscription
{
public:
  typedef std::shared_ptr<SampleSubscription> Ptr;

  SampleSubscription(std::string channel, const ParameterGroup::Map& ping_parameters, std::string sample_data_type);

  void addData(std::shared_ptr<std::vector<unsigned char> >& data) override;

  void addCallback(std::function<void(std::shared_ptr<SampleSet>)> callback);

private:            
  std::vector<std::function<void(std::shared_ptr<SampleSet>)> > callbacks_;
  std::mutex callbacks_mutex_;
  std::string sample_data_type_;
};

} // namespace simrad

#endif
