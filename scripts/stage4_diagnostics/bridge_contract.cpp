// No Gazebo world/physics or ROS node: actual native plugin advertisements and installed converters.
#include "../contact_observer/observer.cpp"
#include <ros_gz_bridge/convert/ros_gz_interfaces.hpp>
#include <ros_gz_bridge/convert/std_msgs.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>
#include <std_msgs/msg/string.hpp>
#include <thread>
#include <algorithm>
#include <iostream>
#include <memory>
namespace ros_gz_bridge {
class FactoryInterface;
std::shared_ptr<FactoryInterface> get_factory(const std::string &,const std::string &);
}
void require(bool value,const char*msg){if(!value)throw std::runtime_error(msg);}
int main(){
 require(bool(ros_gz_bridge::get_factory("ros_gz_interfaces/msg/Contacts","ignition.msgs.Contacts")),"Contacts factory unavailable");
 require(bool(ros_gz_bridge::get_factory("std_msgs/msg/String","ignition.msgs.StringMsg")),"StringMsg factory unavailable");
 ignition::msgs::Contacts contacts;contacts.mutable_header()->mutable_stamp()->set_sec(10);
 auto tag=contacts.mutable_header()->add_data();tag->set_key("frame_id");tag->add_value("b2w_evaluation_seq_42");
 auto c=contacts.add_contact();c->mutable_collision1()->set_name("b2w::body::collision");c->mutable_collision2()->set_name("wall");
 ros_gz_interfaces::msg::Contacts converted;ros_gz_bridge::convert_gz_to_ros(contacts,converted);
 require(converted.header.frame_id=="b2w_evaluation_seq_42" && converted.header.stamp.sec==10,"Contact sequence/stamp lost");
 require(converted.contacts.size()==1 && converted.contacts[0].collision1.name=="b2w::body::collision","Contact identity lost");
 ignition::msgs::StringMsg inventory;inventory.set_data("{\"sequence\":42}");std_msgs::msg::String text;ros_gz_bridge::convert_gz_to_ros(inventory,text);require(text.data==inventory.data(),"Inventory payload changed");
 b2w_evaluation::Observer actual_plugin;ignition::transport::Node node;std::vector<std::string> topics;
 for(int i=0;i<10;++i){node.TopicList(topics);if(topics.size()>=2)break;std::this_thread::sleep_for(std::chrono::milliseconds(50));}
 require(std::find(topics.begin(),topics.end(),"/b2w/evaluation/full_contacts")!=topics.end(),"Native Contacts topic absent");
 require(std::find(topics.begin(),topics.end(),"/b2w/evaluation/collision_inventory")!=topics.end(),"Native inventory topic absent");
 std::cout<<"PASS factories, sequence/stamp/contact/string conversion, actual Observer constructor native advertisements; no ROS node, Gazebo world or physical coverage claim\n";
}
