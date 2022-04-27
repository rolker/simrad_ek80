#ifndef SIMRAD_EK80_SAMPLE_H
#define SIMRAD_EK80_SAMPLE_H

#include <list>
#include <simrad_ek80/subscription.h>
#include <simrad_ek80/utilities.h>
#include <simrad_ek80/callbacks.h>

namespace simrad
{

class Channel;

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

class SampleSubscription: public Subscription, public Callbacks<std::shared_ptr<SampleSet> >
{
public:
  typedef std::shared_ptr<SampleSubscription> Ptr;

  SampleSubscription(std::shared_ptr<Channel> channel, std::string sample_data_type);

  static Ptr subscribe(std::shared_ptr<Channel> channel, float range = 250.0, std::string sample_data_type = "Power");

  void addData(std::shared_ptr<std::vector<unsigned char> >& data) override;

private:            

  std::string sample_data_type_;
  std::shared_ptr<Channel> channel_;
};

} // namespace simrad

#endif
