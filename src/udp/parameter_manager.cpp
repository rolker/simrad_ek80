#include <simrad_ek80/udp/parameter_manager.h>
#include <iostream>

namespace simrad
{

ParameterManager::ParameterManager(Connection::Ptr& connection)
  :connection_(connection)
{
}

ParameterManager::~ParameterManager()
{
  for(auto p: parameter_ids_)
  {
    Request req (connection_,"ParameterServer","CloseSubscription");
    req.addArgument("paramName", p.first);
    req.addArgument("cookie", std::to_string(p.second));
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
  req.addArgument("requestedPort", std::to_string(getPort()));
                    
  Response r(req.getResponse());

  int id;
  std::stringstream ss(r.getArgument("cookie"));
  ss >> id;

  std::lock_guard<std::mutex> lock(parameters_mutex_);

  parameters_[id] = ret;
  parameter_ids_[parameter_name] = id;
  return ret;
}

Parameter::Ptr ParameterManager::get(const std::string& parameter_name)
{
  std::lock_guard<std::mutex> lock(parameters_mutex_);
  if (parameter_ids_.find(parameter_name) != parameter_ids_.end())
  {
    auto pid = parameter_ids_[parameter_name];
    if(parameters_.find(pid) != parameters_.end())
      return parameters_[parameter_ids_[parameter_name]];
  }
  return {};
}

Parameter::Info ParameterManager::getInfo(const std::string& parameter_name)
{
  Request req(connection_, "ParameterServer", "GetParamInfo");
  req.addArgument("paramName", parameter_name);
  Response r(req.getResponse());
  //std::cout << parameter_name << " getInfo response: " << std::string(r) << std::endl;
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
  //std::cout << "stype: " << stype << std::endl;
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

  //std::cout << info.name << ", " << info.description << ", " << info.type << std::endl;

  return info;
}

std::string ParameterManager::getValue(const std::string& parameter_name)
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

void ParameterManager::receivePacket(const std::vector<uint8_t>& packet)
{
  packet::ParameterMessageHeader* h = (packet::ParameterMessageHeader*)(packet.data());
  //std::cerr << "\tMsg: " << h->currentMsg << " of " << h->totalMsg << "\tSeq No: " << h->seqNo << "\tid:" << h->msgID << std::endl;
  if(h->currentMsg > 1)
    throw(Exception("Fragmented update packet not yet supported!"));

  TimePoint latest_update_time = TimePoint();


  switch(h->msgID)
  {
  case 6: //PM_VALUE_ATTRIBUTE_UPDATE = 6
    {
      //std::cout << "Updates..." << std::endl;
      packet::RecordHeader* rec = h->records();
      bool done = false;
      while(!done)
      {
        //std::cout << "Rec Header id: " << rec->recID << " len: " << rec->recLen << std::endl;
        switch (rec->recID)
        {
          case 11: // value update
          {
            packet::ParameterRecord* pr = reinterpret_cast<packet::ParameterRecord*>(rec);
            //std::cerr << "\tValue update with " << pr->paramCount << " records." << std::endl;
            packet::ParameterDefinition* param = pr->parameters();
            for(int i = 0; i<pr->paramCount; ++i)
            {
              ParameterUpdate update;
              update.id = param->cookie;
              update.time = fromSimradTime(param->timeStamp);
              if (update.time > latest_update_time)
                latest_update_time = update.time;
              auto value = param->value();
              char* v = value->value();
              update.value = std::vector<char>(v,v+value->len);
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
              param = param->next();
            }
            break;
          }
          case 12: // attribute update
          {
            // std::cerr << "\tAttribue update" << std::endl;
            // packet::AttributeRecord* ar = reinterpret_cast<packet::AttributeRecord*>(rec);
            // std::cerr << ar->attribCount << " updates" << std::endl;
            // packet::AttributeDefinition* ad = ar->attributes();
            // for(int i = 0; i < ar->attribCount; i++)
            // {
            //   std::cout << "  cookie: " << ad->cookie << std::endl;
            //   std::lock_guard<std::mutex> lock(parameters_mutex_);
            //   auto p = parameters_[ad->cookie];
            //   if(!p)
            //     std::cerr << "Parameter not found with ID " << ad->cookie << " for attribute updates" << std::endl;
            //   else
            //     std::cout << "Attribute updates for " << p->getInfo().name << std::endl;
            //   std::cout << "  update " <<  ad->attributeName.str() << ": " << ad->attributeLen() << " bytes" << std::endl;
            //   ad = ad->next();
            // }
            break;
          }
          case 0xdead: // end of records
            done = true;
            break;
          default:
            throw(Exception("Unknown update record type"));
            break;
        }
        rec = rec->next();
      }
    }
    break;
  default:
    std::cout << "Unhandled parameter update msgID: " << h->msgID << std::endl;
  }
  if (latest_update_time != TimePoint())
    callCallbacks(latest_update_time);
}

} // namespace simrad
