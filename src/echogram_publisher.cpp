#include "echogram_publisher.h"

EchogramPublisher::EchogramPublisher(simrad::Channel::Ptr channel, float range, float range_start, float bin_size, bool replay, std::string frame_id, std::shared_ptr<rosbag::Bag> bag)
  :replay_(replay), frame_id_(frame_id), bag_(bag)
{
  ros::NodeHandle nh;
  topic_ = channel->topicName()+"/echogram";

  sonar_image_pub_ = nh.advertise<marine_acoustic_msgs::RawSonarImage>(topic_, 10);
  if(range > 0.0)
  {
    echogram_sub_ = channel->subscribeToEchogram(range, range_start, bin_size);
    echogram_callback_ = echogram_sub_->addCallback(std::bind(&EchogramPublisher::ping_callback, this, std::placeholders::_1));
  }
}

void EchogramPublisher::ping_callback(std::shared_ptr<simrad::SampleSet> ping)
{
  ros::Time rt(std::chrono::duration_cast<std::chrono::nanoseconds>(ping->time.time_since_epoch()).count()/1000000000.0);
  if(replay_)
    rt = ros::Time::now();

  auto sample_count = ping->samples.size();
  marine_acoustic_msgs::RawSonarImage si;
  si.header.frame_id = frame_id_;
  si.header.stamp = rt;
  si.ping_info.frequency = ping->frequency;
  si.ping_info.sound_speed = ping->soundSpeed;

  auto first_sample_time = ping->start_range/ping->soundSpeed;
  si.sample0 = first_sample_time/ping->sampleInterval;
  
  si.ping_info.rx_beamwidths.push_back(ping->beamWidthX*M_PI/180.0);
  si.ping_info.tx_beamwidths.push_back(ping->beamWidthY*M_PI/180.0);

  // "Positive numbers denotes the fore and starboard directions"
  // Ek80 interface doc, p.196
  si.rx_angles.push_back(ping->directionX*M_PI/180.0);
  si.tx_angles.push_back(ping->directionY*M_PI/180.0);
  si.sample_rate = 1.0/ping->sampleInterval;
  si.tx_delays.push_back(0.0);
  si.samples_per_beam = sample_count;
  si.image.beam_count = 1;
  si.image.dtype = marine_acoustic_msgs::SonarImageData::DTYPE_FLOAT32;
  si.image.data.resize(4*sample_count);
  std::vector<float> samples(sample_count);
  for(int i = 0; i < sample_count; i++)
    samples[i] = simrad::rawToDB(ping->samples[i]);
  memcpy(si.image.data.data(), samples.data(), si.image.data.size());
  sonar_image_pub_.publish(si);
  if(bag_)
    bag_->write(topic_, si.header.stamp, si);
}
