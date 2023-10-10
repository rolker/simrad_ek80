#include <ros/ros.h>
#include <simrad_ek80/udp/server_manager.h>
#include <simrad_ek80/udp/client.h>
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rosgraph_msgs/Clock.h>
#include <rosbag/bag.h>
#include "ping_publisher.h"
#include "echogram_publisher.h"

#include <simrad_ek80/EchogramSubscribe.h>
#include <simrad_ek80/ListChannels.h>
#include <simrad_ek80/ServerInfo.h>
#include <simrad_ek80/StatusInfo.h>
#include <std_srvs/SetBool.h>

std::shared_ptr<simrad::ServerManager> server_manager;
std::shared_ptr<simrad::Client> client;

std::shared_ptr<simrad::ParameterManager> parameter_manager;
std::shared_ptr<simrad::ParameterManager::callback_type> parameter_updates_callback_ptr;

int id = -1;
float range= 0.0;

bool replay = false;

ros::Publisher position_pub;
ros::Publisher orientation_pub;
ros::Publisher velocity_pub;

std::string frame_id = "ek80";
std::string nav_frame_id = "ek80_nav";

std::string bag_file;
std::shared_ptr<rosbag::Bag> bag;

std::vector<std::shared_ptr<PingPublisher> > ping_publishers;

std::map<std::string, std::shared_ptr<EchogramPublisher> > echogram_publishers;


void parameter_update_callback(simrad::TimePoint time)
{
  //std::cerr << "time: " << std::chrono::duration_cast<std::chrono::seconds>(time.time_since_epoch()).count() << std::endl;
  ros::Time rt(std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count()/1000000000.0);
  if(replay)
    rt = ros::Time::now();

  sensor_msgs::NavSatFix nsf;
  nsf.header.stamp = rt;
  nsf.header.frame_id = nav_frame_id;
  nsf.latitude = parameter_manager->get("OwnShip/Latitude")->get<double>(-999, time);
  nsf.longitude =  parameter_manager->get("OwnShip/Longitude")->get<double>(-999, time);
  nsf.altitude = parameter_manager->get("OwnShip/Heave")->get<double>(0, time);

  if(nsf.latitude != -90.0 && nsf.latitude <= 90.0 && nsf.longitude >= -180.0 && nsf.longitude <= 180.0)
  {
    position_pub.publish(nsf);
    if(bag)
      bag->write("position", nsf.header.stamp, nsf);
  }

  tf2::Quaternion q;
  q.setRPY(parameter_manager->get("OwnShip/Roll")->get<double>(0, time)*M_PI/180.0, parameter_manager->get("OwnShip/Pitch")->get<double>(0, time)*M_PI/180.0, (90.0-parameter_manager->get("OwnShip/Heading")->get<double>(0, time))*M_PI/180.0);

  sensor_msgs::Imu imu;
  imu.header.stamp = rt;
  imu.header.frame_id = nav_frame_id;
  imu.orientation = tf2::toMsg(q);
  orientation_pub.publish(imu);
  if(bag)
    bag->write("orientation", imu.header.stamp, imu);

  geometry_msgs::TwistStamped ts;
  ts.header.stamp = rt;
  ts.header.frame_id = nav_frame_id;

  double course_rad_yaw = (90.0-parameter_manager->get("OwnShip/Course")->get<double>(0, time))*M_PI/180.0;
  double sog = parameter_manager->get("OwnShip/Speed")->get<double>(0, time);
  ts.twist.linear.x = sog*cos(course_rad_yaw);
  ts.twist.linear.y = sog*sin(course_rad_yaw);

  velocity_pub.publish(ts);
  if(bag)
    bag->write("velocity", ts.header.stamp, ts);

  //std::cout << "sog: " << parameter_manager->get("OwnShip/Speed")->get<double>(0, time) << ", cog: " << parameter_manager->get("OwnShip/Course")->get<double>(0, time) << std::endl;
}

void server_manager_callback(ros::WallTimerEvent event)
{
  if(!client)
  {
    auto servers = server_manager->getList();
    if(servers.empty())
      ROS_WARN_STREAM_DELAYED_THROTTLE(5.0,"No servers found.");
    for(auto s: servers)
    {
      std::cout << s.string() << std::endl;
      if (!client && (id < 0 || s.getApplicationID() == id))
      {
        client = std::make_shared<simrad::Client>(s);
        client->connect();
        auto channels = client->getChannels();
        for(auto c: channels)
        {
          std::cout << "Channel: " << c->name() << std::endl;
          //ping_publishers.push_back(std::make_shared<PingPublisher>(c, range, replay, frame_id, bag));
        }
        parameter_manager = client->getParameterManager();
        parameter_updates_callback_ptr = parameter_manager->addCallback(&parameter_update_callback);
      }
    }
  }
}

bool getServerInfoService(simrad_ek80::ServerInfo::Request &request, simrad_ek80::ServerInfo::Response &response)
{
  if(client)
  {
    const simrad::Server& server = client->server();
    response.application_type = server.getApplicationType();
    response.application_name = server.getApplicationName();
    response.application_description = server.getApplicationDescription();
    response.application_id = server.getApplicationID();
    response.command_port = server.getCommandPort();
    response.mode = server.getMode();
    response.host_name = server.getHostName();
    response.server_ip = server.ipAddressString();
    response.server_port = server.serverPort();
    return true; 
  }
  return false;
}

bool listChannelsService(simrad_ek80::ListChannels::Request &request, simrad_ek80::ListChannels::Response &response)
{
  if(client)
  {
    auto channels = client->getChannels();
    for(auto channel: channels)
    {
      if(channel)
        response.channel_names.push_back(channel->topicName()+": "+channel->name());
    }
    return true;
  }
  return false;
}

bool statusService(simrad_ek80::StatusInfo::Request &request, simrad_ek80::StatusInfo::Response &response)
{
  if(client)
  {
    auto pm = client->getParameterManager();
    if(pm)
    {
      response.operating_mode = pm->getValue("OperationControl/MainOperation");
      auto pinging = pm->getValue("OperationControl/PlayState");
      response.pinging = pinging == "Running";
      response.ping_mode = pm->getValue("AcousticDeviceSynchroniser/PingMode");
      response.ping_interval = pm->get("AcousticDeviceSynchroniser/Interval")->get<int>(0);
      response.recording = pm->get("SounderStorageManager/SaveRawData")->get<int>(0);
      response.sample_range = pm->get("SounderStorageManager/SampleRange")->get<int>(0);
      response.sample_range_auto = pm->get("SounderStorageManager/SampleRangeAuto")->get<int>(0);
      response.using_per_channel_recording_ranges = pm->get("SounderStorageManager/IndividualChannelRecordingRange")->get<int>(0);
      return true;
    }
  }
  return false;
}

bool enablePingingService(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response)
{
  if(client)
  {
    auto pm = client->getParameterManager();
    if(pm)
    {
      int pinging = request.data;
      pm->set("OperationControl/PlayState", std::to_string(pinging), 3);
      response.success = true;
      return true;
    }
    response.success = false;
    response.message = "Parameter Manager is null";
    return true;
  }
  response.success = false;
  response.message = "Client is null";
  return true;
}

bool echogramSubscribeService(simrad_ek80::EchogramSubscribe::Request &request, simrad_ek80::EchogramSubscribe::Response& response)
{
  if(client)
  {
    auto echo_sub_iterator = echogram_publishers.find(request.channel);
    if(echo_sub_iterator == echogram_publishers.end())
    {
      auto channels = client->getChannels();
      for(auto channel: channels)
        if(channel->topicName() == request.channel)
        {
          echogram_publishers[channel->topicName()] = std::make_shared<EchogramPublisher>(channel, request.range, request.range_start, request.bin_size, replay, frame_id, bag);
          response.success = true;
          return true;
        }
      response.success = false;
      response.message = "Channel "+ request.channel +"not found";
    }
    response.success = false;
    response.message = "Already subscribed";
    return true;
  }
  response.success = false;
  response.message = "Client is null";
  return true;

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
  ROS_INFO_STREAM("range: " << range);
  replay = ros::param::param("~replay", replay);
  ROS_INFO_STREAM("replay? " << replay);

  bag_file = ros::param::param("~bag_file", bag_file);

  if(!bag_file.empty())
  {
    bag = std::make_shared<rosbag::Bag>();
    bag->open(bag_file, rosbag::bagmode::Write);
  }

  position_pub = nh.advertise<sensor_msgs::NavSatFix>("position", 10);
  orientation_pub = nh.advertise<sensor_msgs::Imu>("orientation", 10);
  velocity_pub = nh.advertise<geometry_msgs::TwistStamped>("velocity", 10);

  server_manager = std::make_shared<simrad::ServerManager>(addresses);

  ros::WallTimer server_manager_timer = nh.createWallTimer(ros::WallDuration(.5), &server_manager_callback);

  ros::NodeHandle private_nh("~");

  ros::ServiceServer get_server_info_service = private_nh.advertiseService("get_server_information", &getServerInfoService);
  ros::ServiceServer list_channels_service = private_nh.advertiseService("list_channels", &listChannelsService);
  ros::ServiceServer status_service = private_nh.advertiseService("status", &statusService);
  ros::ServiceServer enable_pinging_service = private_nh.advertiseService("enable_pinging", &enablePingingService);
  ros::ServiceServer echogram_subscribe_service = private_nh.advertiseService("echogram_subscribe", &echogramSubscribeService);

  ros::spin();

  client.reset();

  return 0;
}
