#ifndef SIMRAD_EK80_FILE_EK80_H
#define SIMRAD_EK80_FILE_EK80_H

#include <simrad_ek80/file/channel.h>
#include <simrad_ek80/file/transceiver.h>
#include <simrad_ek80/file/transducer.h>
#include <simrad_ek80/file/format.h>
#include <simrad_ek80/utilities.h>
#include <string>
#include <map>

namespace simrad
{

namespace file
{

class EK80
{
public:
  struct Ping
  {
    TimePoint timestamp;
    std::vector<float> samples;
    double beamwidth_alongship;
    double beamwidth_athwartship;
    double sample_interval;
    double sound_speed;
  };

  void processXML(std::string xml);
  Ping samplesToPowers(const SampleDatagram3& samples) const;
  void processMRU(const MRUDatagram& mru);

  const float& heading() const;
  const float& pitch() const;
  const float& roll() const;
  const float& heave() const;
private:
  std::map<std::string, Channel> channels_;
  std::map<std::string, Transceiver> transceivers_;
  std::map<std::string, Transducer> transducers_;
  float heading_;
  float pitch_;
  float roll_;
  float heave_;
};

} // namespace file
} // namespace simrad

#endif
