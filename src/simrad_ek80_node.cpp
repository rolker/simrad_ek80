#include <ros/ros.h>
#include <simrad_ek80/server_manager.h>
#include <simrad_ek80/client.h>

std::shared_ptr<simrad::ServerManager> server_manager;
std::shared_ptr<simrad::Client> client;
int id = -1;

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
        break;
      }
    }
  }
  else
  {
    auto platform = client->getPlatform();
    auto param_groups = platform->getParameters();
    for(auto params: param_groups)
    {
      std::cout << params.first << std::endl;
      auto pm = params.second->parameters();
      for(auto param: pm)
      {
        std::cout << "  " << param.first << ": ";
        auto i = param.second->getInfo();
        std::cout << i.name << ", " << i.description;
        std::cout << std::endl;
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

  server_manager = std::make_shared<simrad::ServerManager>(addresses);

  ros::Timer server_manager_timer = nh.createTimer(ros::Duration(.5), &server_manager_callback);

  ros::spin();

  return 0;
}
