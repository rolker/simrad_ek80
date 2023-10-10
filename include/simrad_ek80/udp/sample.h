#ifndef SIMRAD_EK80_SAMPLE_H
#define SIMRAD_EK80_SAMPLE_H

#include <list>
#include <simrad_ek80/udp/subscription.h>
#include <simrad_ek80/utilities.h>
#include <simrad_ek80/udp/callbacks.h>

namespace simrad
{

class Channel;

struct SampleSet
{
  TimePoint time;
  float frequency;
  float z1;
  double range;
  double start_range;
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

  SampleSubscription(const Channel& channel, int range, std::string sample_data_type, int start_range);

  //static Ptr subscribe(std::shared_ptr<Channel> channel, int range = 250, std::string sample_data_type = "Power", int start_range=10);

  void addData(std::shared_ptr<std::vector<unsigned char> >& data) override;

private:            
  int range_;
  int start_range_;
  std::string sample_data_type_;
  const Channel& channel_;
};

class EchogramSubscription: public Subscription, public Callbacks<std::shared_ptr<SampleSet> >
{
public:
  EchogramSubscription(const Channel& channel, int range, int start_range, float bin_size, std::string tvg_type = "Sv");

  void addData(std::shared_ptr<std::vector<unsigned char> >& data) override;

private:            
  const Channel& channel_;
};

} // namespace simrad

#endif
