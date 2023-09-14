#ifndef PING_PUBLISHER_H
#define PING_PUBLISHER_H

#include <simrad_ek80/udp/channel.h>
#include <marine_acoustic_msgs/RawSonarImage.h>
#include <ros/ros.h>
#include <rosbag/bag.h>

class PingPublisher
{
public:
  PingPublisher(simrad::Channel::Ptr channel, float range, bool replay, std::string frame_id, std::shared_ptr<rosbag::Bag> bag);

private:
  void ping_callback(std::shared_ptr<simrad::SampleSet> ping);

  std::shared_ptr<simrad::SampleSubscription> sample_power_sub_;
  std::shared_ptr<simrad::SampleSubscription> sample_tvg20_sub_;
  std::shared_ptr<simrad::SampleSubscription::callback_type> power_callback_;
  std::shared_ptr<simrad::SampleSubscription::callback_type> tvg20_callback_;
  std::map<std::string, std::shared_ptr<simrad::SampleSet> > latest_pings_;

  ros::Publisher sonar_image_pub_;
  std::string topic_;
  std::string frame_id_ = "ek80";
  
  std::shared_ptr<rosbag::Bag> bag_;

  bool replay_;

};

#endif
