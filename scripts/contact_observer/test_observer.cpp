#include "coverage.hh"
#include <ignition/gazebo/components/World.hh>
#include <ignition/gazebo/components/Model.hh>
#include <ignition/gazebo/components/Link.hh>
#include <cassert>
using namespace ignition::gazebo;
int main() {
  EntityComponentManager ecm;
  auto world=ecm.CreateEntity();ecm.CreateComponent(world,components::World());ecm.CreateComponent(world,components::Name("course_world"));
  auto model=ecm.CreateEntity();ecm.CreateComponent(model,components::Model());ecm.CreateComponent(model,components::Name("b2w"));ecm.CreateComponent(model,components::ParentEntity(world));
  auto link=ecm.CreateEntity();ecm.CreateComponent(link,components::Link());ecm.CreateComponent(link,components::Name("base_link"));ecm.CreateComponent(link,components::ParentEntity(model));
  auto ground=ecm.CreateEntity();ecm.CreateComponent(ground,components::Name("ground"));
  std::vector<Entity> ids;
  for(int i=0;i<26;i++) {
    auto e=ecm.CreateEntity();ids.push_back(e);ecm.CreateComponent(e,components::Collision());ecm.CreateComponent(e,components::Name("collision_"+std::to_string(i)));ecm.CreateComponent(e,components::ParentEntity(link));
  }
  auto names=b2w_evaluation::Request(ecm);
  assert(names.size()==26);
  for(auto e:ids) assert(ecm.Component<components::ContactSensorData>(e));
  auto empty=b2w_evaluation::Read(ecm);
  assert(empty.contact_size()==0);
  for(auto e:ids) {
    ignition::msgs::Contacts input;
    auto contact=input.add_contact();contact->mutable_collision1()->set_id(e);contact->mutable_collision2()->set_id(ground);
    ecm.Component<components::ContactSensorData>(e)->Data()=input;
    auto output=b2w_evaluation::Read(ecm);
    assert(output.contact_size()==1);
    assert(output.contact(0).collision1().name().find("course_world::b2w::base_link::collision_")==0);
    ecm.Component<components::ContactSensorData>(e)->Data().Clear();
  }
  // A transient contact between publication ticks must survive until drain.
  b2w_evaluation::Accumulator accumulator;
  ignition::msgs::Contacts short_contact;
  auto transient=short_contact.add_contact();transient->mutable_collision1()->set_id(ids[0]);transient->mutable_collision2()->set_id(ground);
  ecm.Component<components::ContactSensorData>(ids[0])->Data()=short_contact;
  accumulator.Sample(ecm);
  ecm.Component<components::ContactSensorData>(ids[0])->Data().Clear();
  for(int i=0;i<19;i++) accumulator.Sample(ecm);
  auto interval=accumulator.Drain();
  assert(interval.contact_size()==1);
  // Losing the first publication cannot erase the veto in later heartbeats.
  assert(accumulator.Drain().contact_size()==1);
  assert(accumulator.Forbidden());
  return 0;
}
