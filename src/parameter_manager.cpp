#include <simrad_ek80/parameter_manager.h>
#include <iostream>

namespace simrad
{

ParameterManager::ParameterManager(Connection::Ptr& connection, SubscriptionPort::Ptr& subscription_port)
  :connection_(connection), subscription_port_(subscription_port)
{
  if(!subscription_port_)
    subscription_port_ = std::make_shared<SubscriptionPort>();
  subscription_port_->addCallback([this](const std::vector<uint8_t>& packet){this->process_packet(packet);});
}

ParameterManager::~ParameterManager()
{
  for(auto p: parameters_)
  {
    Request req (connection_,"ParameterServer","CloseSubscription");
    req.addArgument("paramName", parameter_names_[p.first]);
    req.addArgument("cookie", std::to_string(p.first));
    req.getResponse();
  }
}

Parameter::Ptr ParameterManager::subscribe(const std::string& parameter_name, bool use_default)
{
  bool value_is_time = false;
  //if (!boost::find_last(paramName,"/PingTime").empty())
  //  value_is_time = true;

  Parameter::Ptr ret = std::make_shared<Parameter>(getInfo(parameter_name), use_default, value_is_time);
                    
  Request req (connection_, "ParameterServer", "Subscribe");
  req.addArgument("paramName", parameter_name);
  req.addArgument("requestedPort", std::to_string(subscription_port_->getPort()));
                    
  Response r(req.getResponse());

  int id;
  std::stringstream ss(r.getArgument("cookie"));
  ss >> id;

  std::lock_guard<std::mutex> lock(parameters_mutex_);

  parameters_[id] = ret;
  parameter_names_[id] = parameter_name;
  return ret;
}

Parameter::Info ParameterManager::getInfo(const std::string& parameter_name)
{
  Request req(connection_, "ParameterServer", "GetParamInfo");
  req.addArgument("paramName", parameter_name);
  Response r(req.getResponse());
  switch (r.getErrorCode())
  {
  case 0:
    // no problem!
    break;
  case -2147467259:
    // ER60 parameter not found
    throw(InvalidParameterException(r.getErrorMessage()));
  default:
    std::cerr << static_cast<std::string>(r) << std::endl;
    throw(Exception(r.getErrorMessage()));
  }
    
  Parameter::Info info;
    
  std::string stype = r.getArgument("paramInfo/attributes/Type");
  if (stype.empty()) // handle silly Simrad inconsistencies between ME70 and ER60 software
  {
    // ER60
    stype = r.getArgument("paramInfo/attributes/type");
    info.name = r.getArgument("paramInfo/attributes/name");
    info.description = r.getArgument("paramInfo/attributes/description");
    info.defaultValue = r.getArgument("paramInfo/attributes/default");
  }
  else
  {
    info.name = r.getArgument("paramInfo/attributes/Name");
    info.description = r.getArgument("paramInfo/attributes/Description");
    info.defaultValue = r.getArgument("paramInfo/attributes/Default");
  }

  std::stringstream ss(stype);
  ss >> info.type;
    
  return info;
}

std::string ParameterManager::get(const std::string& parameter_name)
{
  Request req(connection_, "ParameterServer", "GetParameter");
  req.addArgument("paramName", parameter_name);
  req.addArgument("time", "0");
  Response r(req.getResponse());
  switch (r.getErrorCode())
  {
  case 0:
    // no problem!
    break;
  case -2147467259:
    // ER60 parameter not found
    throw(InvalidParameterException(r.getErrorMessage()));
  default:
    std::cerr << static_cast<std::string>(r) << std::endl;
    throw(Exception(r.getErrorMessage()));
  }
                    
  if(r.getErrorCode())
    throw(Exception(r.getErrorMessage()));
                    
  return r.getArgument("paramValue/value");
}

void ParameterManager::process_packet(const std::vector<uint8_t>& packet)
{
                    
  #pragma pack(1)
                    
  struct Record
  {
      unsigned short recID;
      unsigned short recLen;
  };
  
  struct Header
  {
      unsigned short currentMsg;
      unsigned short totalMsg;
      int32_t seqNo;
      int32_t msgID;
      Record firstRecord;
  };
  
  struct Value
  {
      int32_t len;
      char value;
      int Size(){return len+sizeof(len);}
  };
  
  struct ParameterDefinition
  {
      int32_t cookie;
      uint64_t timeStamp;
      Value value;
      int32_t unused;
      int Size(){return sizeof(ParameterDefinition) - sizeof(value) + value.Size();}
  };
  
  struct ParameterRecord: public Record
  {
      int32_t paramCount;
      ParameterDefinition parameters;
  };
  
  #pragma pack()
                    
  int cursor = sizeof(Header);
  
  Header *h = (Header*)(packet.data());
  //std::cerr << "\tMsg: " << h->currentMsg << " of " << h->totalMsg << "\tSeq No: " << h->seqNo << "\tid:" << h->msgID << std::endl;
  if(h->currentMsg > 1)
    throw(Exception("Fragmented update packet not yet supported!"));
                    
  if(h->msgID == 6) //PM_VALUE_ATTRIBUTE_UPDATE = 6
  {
    Record *rec = &h->firstRecord;
    bool done = false;
    while(!done)
    {
      switch (rec->recID)
      {
        case 11: // value update
        {
          ParameterRecord *pr = (ParameterRecord *)rec;
          //std::cerr << "\tValue update with " << pr->paramCount << " records." << std::endl;
          ParameterDefinition *param = &pr->parameters;
          for(int i = 0; i<pr->paramCount; ++i)
          {
              //std::cerr << "ID:\t" << param->cookie << " time: " << gz4d::Time::FromNT(param->timeStamp).String() << " (" << param->timeStamp << ")" << std::endl;
              //std::cerr << "\tvalue len: " << param->value.len << std::endl;
              
              //if(param->timeStamp)
              {
                ParameterUpdate update;
                update.id = param->cookie;
                update.time = fromSimradTime(param->timeStamp);
                char *v = &param->value.value;
                update.value = std::vector<char>(v,&v[param->value.len]);
                Parameter::Ptr p;
                while(!p)
                {
                  {
                    std::lock_guard<std::mutex> lock(parameters_mutex_);
                    p = parameters_[update.id];
                  }
                  if (!p)
                    std::this_thread::yield();
                }
                p->update(update);
              }
              param =  (ParameterDefinition*)&(((char *)param)[param->Size()]);
          }
          break;
        }
        case 12: // attribute update
          //std::cerr << "\tAttribue update" << std::endl;
          break;
        case 0xdead: // end of records
          done = true;
          break;
        default:
          break;
      }  
      rec = reinterpret_cast<Record*>(&(reinterpret_cast<char *>(rec)[rec->recLen]));
      
    }
  }
}

} // namespace simrad
