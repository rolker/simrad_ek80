#include <ros/ros.h>
#include <simrad_ek80/server_manager.h>
#include <simrad_ek80/client.h>
#include <acoustic_msgs/RawSonarImage.h>

std::shared_ptr<simrad::ServerManager> server_manager;
std::shared_ptr<simrad::Client> client;
std::shared_ptr<simrad::Parameter> latitude_parameter;
std::shared_ptr<simrad::Parameter> longitude_parameter;
std::shared_ptr<simrad::SampleSubscription> sample_power_sub;
std::shared_ptr<simrad::SampleSubscription> sample_tvg20_sub;
std::map<std::string, std::shared_ptr<simrad::SampleSet> > latest_pings;
std::map<std::string, simrad::ParameterGroup::Ptr> channel_params;
int id = -1;

ros::Publisher sonar_image_pub;

void position_update_callback(simrad::TimePoint time)
{
  ros::Time rt(std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()).count()/1000.0);
  std::cout << "position callback: " << std::setprecision(13) << rt.toSec() << std::endl;
  std::cout << "  lat: " << latitude_parameter->get<double>(0.0, time) << " lon: " << longitude_parameter->get<double>(0.0, time) << std::endl;
}

void channel_params_callback(simrad::TimePoint time)
{
  ros::Time rt(std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()).count()/1000.0);  
  std::cout << "channel params callback: " << std::setprecision(13) << rt.toSec() << std::endl;
}

void sound_speed_callback(simrad::TimePoint time)
{
  ros::Time rt(std::chrono::duration_cast<std::chrono::milliseconds>(time.time_since_epoch()).count()/1000.0);  
  std::cout << "sound speed callback: " << std::setprecision(13) << rt.toSec() << std::endl;
}


void ping_callback(std::shared_ptr<simrad::SampleSet> ping)
{
  ros::Time rt(std::chrono::duration_cast<std::chrono::milliseconds>(ping->time.time_since_epoch()).count()/1000.0);
  std::cout << "Ping: " << std::setprecision(13) << rt.toSec() << ", data type: " << ping->dataType << "\n   ";
  for(int i = 0; i < ping->samples.size(); i++)
  {
    std::cout << std::setprecision(4) << simrad::rawToDB(ping->samples[i]) << ", ";
    if (i == 15 && ping->samples.size() -15 > i)
    {
      i = ping->samples.size()-15;
      std::cout << " ... ";
    }
  }
  std::cout << std::endl;

  latest_pings[ping->dataType] = ping;

  if(latest_pings["Power"] && latest_pings["TVG20"] && latest_pings["Power"]->time == latest_pings["TVG20"]->time)
  {
    auto power = latest_pings["Power"];
    auto tvg = latest_pings["TVG20"];
    auto sample_count = std::min(power->samples.size(), tvg->samples.size());
    acoustic_msgs::RawSonarImage si;
    si.header.frame_id = "ek80";
    si.header.stamp = rt;
    si.ping_info.frequency = power->frequency;
    si.ping_info.sound_speed = power->soundSpeed;
    si.ping_info.rx_beamwidths.push_back(power->beamWidthX*M_PI/180.0);
    si.ping_info.tx_beamwidths.push_back(power->beamWidthY*M_PI/180.0);
    si.rx_angles.push_back(power->directionX*M_PI/180.0);
    si.tx_angles.push_back(power->directionY*M_PI/180.0);
    si.sample_rate = 1.0/power->sampleInterval;
    si.tx_delays.push_back(0.0);
    si.samples_per_beam = sample_count;
    si.image.dtype = acoustic_msgs::SonarImageData::DTYPE_FLOAT32;
    si.image.data.resize(4*sample_count);
    std::vector<float> samples(sample_count);
    for(int i = 0; i < sample_count; i++)
      samples[i] = simrad::rawToDB(power->samples[i]) + simrad::rawToDB(tvg->samples[i]);
    memcpy(si.image.data.data(), samples.data(), si.image.data.size());
    sonar_image_pub.publish(si);
  }

}


void server_manager_callback(ros::TimerEvent event)
{
  if(!client)
  {
    auto servers = server_manager->getList();
    for(auto s: servers)
    {
      std::cout << s.string() << std::endl;
      if (id < 0 || s.getID() == id)
      {
        client = std::make_shared<simrad::Client>(s);
        client->connect();
        auto platform = client->getPlatform();
        auto param_groups = platform->getParameters();
        param_groups["position"]->addCallback(&position_update_callback);
        latitude_parameter = param_groups["position"]->parameters()["latitude"];
        longitude_parameter = param_groups["position"]->parameters()["longitude"];
        auto t = client->getTransducer();
        auto channels = t->getChannels();
        for(auto c: channels)
          std::cout << "Channel: " << c->getName() << std::endl;
        sample_power_sub = channels[0]->getSamples(200, "Power");
        sample_power_sub->addCallback(&ping_callback);
        sample_tvg20_sub = channels[0]->getSamples(200, "TVG20");
        sample_tvg20_sub->addCallback(&ping_callback);
        channel_params = channels[0]->getParameters();
        channel_params["channel"]->addCallback(&channel_params_callback);
        channel_params["channel"]->parameters()["soundSpeed"]->addCallback(&sound_speed_callback);
      }
    }
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "simrad_ek80");
  ros::NodeHandle nh;

  std::vector<std::string> addresses;

  if(ros::param::has("~remote_addresses"))
    ros::param::get("~remote_addresses", addresses);

  id = ros::param::param("~id", id);

  sonar_image_pub = nh.advertise<acoustic_msgs::RawSonarImage>("sonar_image", 10);

  server_manager = std::make_shared<simrad::ServerManager>(addresses);

  ros::Timer server_manager_timer = nh.createTimer(ros::Duration(.5), &server_manager_callback);

  ros::spin();

  client.reset();

  return 0;
}
