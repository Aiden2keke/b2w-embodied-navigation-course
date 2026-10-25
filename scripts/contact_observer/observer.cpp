#include "coverage.hh"
#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/msgs/stringmsg.pb.h>
#include <chrono>
#include <iomanip>
#include <sstream>
namespace b2w_evaluation {
class Observer: public ignition::gazebo::System,
                public ignition::gazebo::ISystemPreUpdate,
                public ignition::gazebo::ISystemPostUpdate {
  ignition::transport::Node node;
  ignition::transport::Node::Publisher contacts=node.Advertise<ignition::msgs::Contacts>("/b2w/evaluation/full_contacts");
  ignition::transport::Node::Publisher inventory=node.Advertise<ignition::msgs::StringMsg>("/b2w/evaluation/collision_inventory");
  std::map<ignition::gazebo::Entity,std::string> names;
  double last=-1.;
  Accumulator accumulator;
  unsigned long sequence=0,steps=0;
 public:
  void PreUpdate(const ignition::gazebo::UpdateInfo& info,ignition::gazebo::EntityComponentManager& ecm) override {
    if(!info.paused) names=Request(ecm);
  }
  void PostUpdate(const ignition::gazebo::UpdateInfo& info,const ignition::gazebo::EntityComponentManager& ecm) override {
    const double sim=std::chrono::duration<double>(info.simTime).count();
    if(info.paused) return;
    accumulator.Sample(ecm,sim);++steps;
    if(sim-last < .05) return;
    const auto intervalStart=last;
    last=sim;
    auto data=accumulator.Drain();++sequence;
    const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    data.mutable_header()->mutable_stamp()->set_sec(ns/1000000000);
    data.mutable_header()->mutable_stamp()->set_nsec(ns%1000000000);
    auto tag=data.mutable_header()->add_data();tag->set_key("frame_id");tag->add_value("b2w_evaluation_seq_"+std::to_string(sequence));
    // Empty contact messages are deliberate fresh heartbeats.
    contacts.Publish(data);
    std::ostringstream json;json<<std::setprecision(17)<<"{\"stamp\":"<<sim<<",\"sequence\":"<<sequence<<",\"interval_start\":"<<intervalStart<<",\"sampled_steps\":"<<steps<<",\"ever_forbidden_contact\":"<<(accumulator.Forbidden()?"true":"false")<<",\"last_ground_support_stamp\":"<<accumulator.lastSupport<<",\"collisions\":[";
    bool first=true;
    for(const auto& [id,name]:names) {if(!first)json<<",";first=false;json<<std::quoted(name);}
    json<<"],\"evaluation_only\":true}";
    ignition::msgs::StringMsg msg;msg.set_data(json.str());inventory.Publish(msg);
    steps=0;
  }
};
}
IGNITION_ADD_PLUGIN(b2w_evaluation::Observer,ignition::gazebo::System,
                   ignition::gazebo::ISystemPreUpdate,ignition::gazebo::ISystemPostUpdate)
