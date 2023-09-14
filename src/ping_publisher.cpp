#include "ping_publisher.h"

PingPublisher::PingPublisher(simrad::Channel::Ptr channel, float range, bool replay, std::string frame_id, std::shared_ptr<rosbag::Bag> bag)
  :replay_(replay), frame_id_(frame_id), bag_(bag)
{
  ros::NodeHandle nh;
  auto name = channel->getName();
  auto parts = simrad::split(name, " ");
  topic_ = simrad::replace(parts.back(), '-', '_');

  sonar_image_pub_ = nh.advertise<marine_acoustic_msgs::RawSonarImage>(topic_, 10);
  if(range > 0.0)
  {
    sample_power_sub_ = simrad::SampleSubscription::subscribe(channel, range, "Power");
    power_callback_ = sample_power_sub_->addCallback(std::bind(&PingPublisher::ping_callback, this, std::placeholders::_1));
    sample_tvg20_sub_ = simrad::SampleSubscription::subscribe(channel, range, "TVG20");
    tvg20_callback_ = sample_tvg20_sub_->addCallback(std::bind(&PingPublisher::ping_callback, this, std::placeholders::_1));
  }
}

void PingPublisher::ping_callback(std::shared_ptr<simrad::SampleSet> ping)
{
  ros::Time rt(std::chrono::duration_cast<std::chrono::nanoseconds>(ping->time.time_since_epoch()).count()/1000000000.0);
  if(replay_)
    rt = ros::Time::now();

  latest_pings_[ping->dataType] = ping;

  if(latest_pings_["Power"] && latest_pings_["TVG20"] && latest_pings_["Power"]->time == latest_pings_["TVG20"]->time)
  {
    //std::cout << "Ping! " << std::setprecision(13) << rt.toSec() << std::endl;
    auto power = latest_pings_["Power"];
    auto tvg = latest_pings_["TVG20"];
    auto sample_count = std::min(power->samples.size(), tvg->samples.size());
    marine_acoustic_msgs::RawSonarImage si;
    si.header.frame_id = frame_id_;
    si.header.stamp = rt;
    si.ping_info.frequency = power->frequency;
    si.ping_info.sound_speed = power->soundSpeed;

    si.ping_info.rx_beamwidths.push_back(power->beamWidthX*M_PI/180.0);
    si.ping_info.tx_beamwidths.push_back(power->beamWidthY*M_PI/180.0);

    // "Positive numbers denotes the fore and starboard directions"
    // Ek80 interface doc, p.196
    si.rx_angles.push_back(power->directionX*M_PI/180.0);
    si.tx_angles.push_back(power->directionY*M_PI/180.0);
    si.sample_rate = 1.0/power->sampleInterval;
    si.tx_delays.push_back(0.0);
    si.samples_per_beam = sample_count;
    si.image.beam_count = 1;
    si.image.dtype = marine_acoustic_msgs::SonarImageData::DTYPE_FLOAT32;
    si.image.data.resize(4*sample_count);
    std::vector<float> samples(sample_count);
    for(int i = 0; i < sample_count; i++)
      samples[i] = simrad::rawToDB(power->samples[i]) + simrad::rawToDB(tvg->samples[i]);
    memcpy(si.image.data.data(), samples.data(), si.image.data.size());
    sonar_image_pub_.publish(si);
    if(bag_)
      bag_->write(topic_, si.header.stamp, si);
  }
}
