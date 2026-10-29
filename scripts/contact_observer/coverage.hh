#pragma once
// Evaluation only. Request contact reporting without editing any collision shape.
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/ContactSensorData.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <map>
#include <string>
#include <vector>
namespace b2w_evaluation {
using namespace ignition::gazebo;
inline bool Robot(const Entity e,const EntityComponentManager& ecm) {
  auto name=scopedName(e,ecm,"::",false);
  return name.rfind("course_world::b2w::",0)==0;
}
inline std::map<Entity,std::string> Request(EntityComponentManager& ecm) {
  std::map<Entity,std::string> names;
  std::vector<Entity> missing;
  ecm.Each<components::Collision>([&](const Entity e,const components::Collision*) {
    if(Robot(e,ecm)) {
      auto name=scopedName(e,ecm,"::",false);
      names[e]=name.substr(std::string("course_world::b2w::").size());
      if(!ecm.Component<components::ContactSensorData>(e)) missing.push_back(e);
    }
    return true;
  });
  for(auto e:missing) ecm.CreateComponent(e,components::ContactSensorData());
  return names;
}
inline ignition::msgs::Contacts Read(const EntityComponentManager& ecm) {
  ignition::msgs::Contacts result;
  ecm.Each<components::Collision,components::ContactSensorData>(
    [&](const Entity e,const components::Collision*,const components::ContactSensorData* data) {
      if(Robot(e,ecm)) for(const auto& input:data->Data().contact()) {
        auto output=result.add_contact();*output=input;
        for(auto entity:{output->mutable_collision1(),output->mutable_collision2()}) {
          auto name=scopedName(entity->id(),ecm,"::",false);
          if(!name.empty()) entity->set_name(name);
        }
      }
      return true;
    });
  return result;
}
inline bool Support(const ignition::msgs::Contact& contact) {
  const std::string ground="course_world::aws_robomaker_warehouse_GroundB_01_001::link::collision";
  auto a=contact.collision1().name(),b=contact.collision2().name();
  for(const auto& leg:{"FL","FR","RL","RR"}) {
    const auto foot="course_world::b2w::"+std::string(leg)+"_foot::"+leg+"_foot_collision";
    if((a==foot && b==ground)||(b==foot && a==ground)) return true;
  }
  return false;
}
class Accumulator {
  ignition::msgs::Contacts pending;
  std::map<std::pair<std::string,std::string>,ignition::msgs::Contact> sticky;
 public:
  double lastSupport=-1.;
  void Sample(const EntityComponentManager& ecm,double sim=0.) {
    auto frame=Read(ecm);
    for(const auto& contact:frame.contact()) {
      if(Support(contact)) {*pending.add_contact()=contact;lastSupport=sim;}
      else {
        auto a=contact.collision1().name(),b=contact.collision2().name();
        if(b<a)std::swap(a,b);
        sticky.emplace(std::make_pair(a,b),contact);
      }
    }
  }
  bool Forbidden() const {return !sticky.empty();}
  ignition::msgs::Contacts Drain() {
    auto output=pending;pending.Clear();
    // Vetoes persist for the whole simulator session. Dropping a report cannot
    // erase a transient collision when a later fresh heartbeat is received.
    for(const auto& [pair,contact]:sticky)*output.add_contact()=contact;
    return output;
  }
};
}
