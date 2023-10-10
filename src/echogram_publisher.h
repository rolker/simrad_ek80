#ifndef ECHOGRAM_PUBLISHER_H
#define ECHOGRAM_PUBLISHER_H

#include <simrad_ek80/udp/channel.h>
#include <marine_acoustic_msgs/RawSonarImage.h>
#include <ros/ros.h>
#include <rosbag/bag.h>

class EchogramPublisher
{
public:
  EchogramPublisher(simrad::Channel::Ptr channel, float range, float range_start, float bin_size, bool replay, std::string frame_id, std::shared_ptr<rosbag::Bag> bag);

private:
  void ping_callback(std::shared_ptr<simrad::SampleSet> ping);

  std::shared_ptr<simrad::EchogramSubscription> echogram_sub_;
  std::shared_ptr<simrad::EchogramSubscription::callback_type> echogram_callback_;

  ros::Publisher sonar_image_pub_;
  std::string topic_;
  std::string frame_id_ = "ek80";
  
  std::shared_ptr<rosbag::Bag> bag_;

  bool replay_;

};

#endif
