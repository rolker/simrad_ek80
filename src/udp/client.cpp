#include <simrad_ek80/udp/client.h>

namespace simrad
{

Client::Client(Server const &s)
  :server_(s)
{
}

Client::Ptr Client::create(Server const &s)
{
  Ptr ret(new  Client(s));
  return ret;
}

void Client::connect(std::string uname, std::string pwd)
{
  connection_ = std::make_shared<Connection>();
  connection_->connect(server_.getAddress(), uname, pwd);
    
  parameter_manager_ = std::make_shared<ParameterManager>(connection_);
  subscription_manager_ = std::make_shared<SubscriptionManager>(connection_);
    
  std::string cs = parameter_manager_->getValue("TransceiverMgr/Channels");
  std::vector<std::string> channelList = split(cs, ",");

  for(auto c: channelList)
    channels_[c] = std::make_shared<Channel>(parameter_manager_, subscription_manager_, c);
  
  parameter_manager_->subscribe("OwnShip/Latitude",false);
  parameter_manager_->subscribe("OwnShip/Longitude",false);
  parameter_manager_->subscribe("OwnShip/Heading",false);
  parameter_manager_->subscribe("OwnShip/Speed");
  parameter_manager_->subscribe("OwnShip/Course");
  parameter_manager_->subscribe("OwnShip/Roll");
  parameter_manager_->subscribe("OwnShip/Pitch");
  parameter_manager_->subscribe("OwnShip/Heave");
  parameter_manager_->subscribe("OwnShip/MotionData");

  parameter_manager_->subscribe("TransceiverMgr/Latitude",false);
  parameter_manager_->subscribe("TransceiverMgr/Longitude",false);
  parameter_manager_->subscribe("TransceiverMgr/Heave");
  parameter_manager_->subscribe("TransceiverMgr/Roll");
  parameter_manager_->subscribe("TransceiverMgr/Pitch");

  parameter_manager_->subscribe("OperationControl/MainOperation", false);
  parameter_manager_->subscribe("OperationControl/PlayState", false);
  parameter_manager_->subscribe("AcousticDeviceSynchroniser/PingMode", false);
  parameter_manager_->subscribe("AcousticDeviceSynchroniser/Interval", false);
  //parameter_manager_->subscribe("StateManager/ActivateState", false);
  parameter_manager_->subscribe("SounderStorageManager/SaveRawData", false);
  parameter_manager_->subscribe("SounderStorageManager/SampleRange", false);
  parameter_manager_->subscribe("SounderStorageManager/SampleRangeAuto", false);
  parameter_manager_->subscribe("SounderStorageManager/IndividualChannelRecordingRange", false);

}

std::vector<Channel::Ptr> Client::getChannels()
{
  std::vector<Channel::Ptr> ret;
  for(auto c: channels_)
    ret.push_back(c.second);
  return ret;
}

std::shared_ptr<ParameterManager> Client::getParameterManager()
{
  return parameter_manager_;
}

const Server& Client::server() const
{
  return server_;
}


} // namespace simrad
