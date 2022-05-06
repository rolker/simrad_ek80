#include <ros/ros.h>
#include <simrad_ek80/server_manager.h>
#include <simrad_ek80/client.h>
#include <acoustic_msgs/RawSonarImage.h>
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rosgraph_msgs/Clock.h>

std::shared_ptr<simrad::ServerManager> server_manager;
std::shared_ptr<simrad::Client> client;
std::shared_ptr<simrad::SampleSubscription> sample_power_sub;
std::shared_ptr<simrad::SampleSubscription> sample_tvg20_sub;
std::shared_ptr<simrad::SampleSubscription::callback_type> power_callback;
std::shared_ptr<simrad::SampleSubscription::callback_type> tvg20_callback;
std::map<std::string, std::shared_ptr<simrad::SampleSet> > latest_pings;

std::shared_ptr<simrad::ParameterManager> parameter_manager;
std::shared_ptr<simrad::ParameterManager::callback_type> parameter_updates_callback_ptr;

int id = -1;
float range= 0.0;

bool replay = false;

ros::Publisher sonar_image_pub;
ros::Publisher position_pub;
ros::Publisher orientation_pub;
ros::Publisher velocity_pub;

std::string frame_id = "ek80";
std::string nav_frame_id = "ek80_nav";

void parameter_update_callback(simrad::TimePoint time)
{
  //std::cerr << "time: " << std::chrono::duration_cast<std::chrono::seconds>(time.time_since_epoch()).count() << std::endl;
  ros::Time rt(std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count()/1000000000.0);
  if(replay)
    rt = ros::Time::now();

  sensor_msgs::NavSatFix nsf;
  nsf.header.stamp = rt;
  nsf.header.frame_id = nav_frame_id;
  nsf.latitude = parameter_manager->get("OwnShip/Latitude")->get<double>(0, time);
  nsf.longitude =  parameter_manager->get("OwnShip/Longitude")->get<double>(0, time);
  nsf.altitude = parameter_manager->get("OwnShip/Heave")->get<double>(0, time);

  position_pub.publish(nsf);

  tf2::Quaternion q;
  q.setRPY(parameter_manager->get("OwnShip/Roll")->get<double>(0, time)*M_PI/180.0, parameter_manager->get("OwnShip/Pitch")->get<double>(0, time)*M_PI/180.0, (90.0-parameter_manager->get("OwnShip/Heading")->get<double>(0, time))*M_PI/180.0);

  sensor_msgs::Imu imu;
  imu.header.stamp = rt;
  imu.header.frame_id = nav_frame_id;
  imu.orientation = tf2::toMsg(q);
  orientation_pub.publish(imu);

  geometry_msgs::TwistStamped ts;
  ts.header.stamp = rt;
  ts.header.frame_id = nav_frame_id;

  double course_rad_yaw = (90.0-parameter_manager->get("OwnShip/Course")->get<double>(0, time))*M_PI/180.0;
  double sog = parameter_manager->get("OwnShip/Speed")->get<double>(0, time);
  ts.twist.linear.x = sog*cos(course_rad_yaw);
  ts.twist.linear.y = sog*sin(course_rad_yaw);

  velocity_pub.publish(ts);
  //std::cout << "sog: " << parameter_manager->get("OwnShip/Speed")->get<double>(0, time) << ", cog: " << parameter_manager->get("OwnShip/Course")->get<double>(0, time) << std::endl;
}

void ping_callback(std::shared_ptr<simrad::SampleSet> ping)
{
  ros::Time rt(std::chrono::duration_cast<std::chrono::nanoseconds>(ping->time.time_since_epoch()).count()/1000000000.0);
  if(replay)
    rt = ros::Time::now();

  latest_pings[ping->dataType] = ping;

  if(latest_pings["Power"] && latest_pings["TVG20"] && latest_pings["Power"]->time == latest_pings["TVG20"]->time)
  {
    //std::cout << "Ping! " << std::setprecision(13) << rt.toSec() << std::endl;
    auto power = latest_pings["Power"];
    auto tvg = latest_pings["TVG20"];
    auto sample_count = std::min(power->samples.size(), tvg->samples.size());
    acoustic_msgs::RawSonarImage si;
    si.header.frame_id = frame_id;
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
    si.image.num_beams = 1;
    si.image.dtype = acoustic_msgs::SonarImageData::DTYPE_FLOAT32;
    si.image.data.resize(4*sample_count);
    std::vector<float> samples(sample_count);
    for(int i = 0; i < sample_count; i++)
      samples[i] = simrad::rawToDB(power->samples[i]) + simrad::rawToDB(tvg->samples[i]);
    memcpy(si.image.data.data(), samples.data(), si.image.data.size());
    sonar_image_pub.publish(si);
  }

}


void server_manager_callback(ros::WallTimerEvent event)
{
  if(!client)
  {
    auto servers = server_manager->getList();
    if(servers.empty())
      std::cout << "No servers found." << std::endl;
    for(auto s: servers)
    {
      std::cout << s.string() << std::endl;
      if (!client && (id < 0 || s.getID() == id))
      {
        client = std::make_shared<simrad::Client>(s);
        client->connect();
        auto channels = client->getChannels();
        for(auto c: channels)
          std::cout << "Channel: " << c->getName() << std::endl;
        if(range > 0.0)
        {
          sample_power_sub = simrad::SampleSubscription::subscribe(channels[0], range, "Power");
          power_callback = sample_power_sub->addCallback(&ping_callback);
          sample_tvg20_sub = simrad::SampleSubscription::subscribe(channels[0], range, "TVG20");
          tvg20_callback = sample_tvg20_sub->addCallback(&ping_callback);
        }
        parameter_manager = client->getParameterManager();
        parameter_updates_callback_ptr = parameter_manager->addCallback(&parameter_update_callback);
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
  frame_id = ros::param::param("~frame_id", frame_id);
  range = ros::param::param("~range", range);
  replay = ros::param::param("~replay", replay);
  std::cout << "replay? " << replay << std::endl;

  sonar_image_pub = nh.advertise<acoustic_msgs::RawSonarImage>("sonar_image", 10);
  position_pub = nh.advertise<sensor_msgs::NavSatFix>("position", 10);
  orientation_pub = nh.advertise<sensor_msgs::Imu>("orientation", 10);
  velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("velocity", 10);

  server_manager = std::make_shared<simrad::ServerManager>(addresses);

  ros::WallTimer server_manager_timer = nh.createWallTimer(ros::WallDuration(.5), &server_manager_callback);

  ros::spin();

  client.reset();

  return 0;
}
