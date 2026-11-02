// SPDX-License-Identifier: Apache-2.0
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
namespace gazebo {
class FuelContactObserver:public WorldPlugin {
 transport::NodePtr gz;transport::SubscriberPtr sub;std::unique_ptr<ros::NodeHandle> node;ros::Publisher pub;
 public:void Load(physics::WorldPtr world,sdf::ElementPtr) override {
  node.reset(new ros::NodeHandle());pub=node->advertise<std_msgs::String>("/fuel_evidence/contacts",100);
  gz.reset(new transport::Node());gz->Init(world->Name());sub=gz->Subscribe("~/physics/contacts",&FuelContactObserver::Contacts,this);
 }
 void Contacts(ConstContactsPtr &msg) {
  std::ostringstream s;s.precision(17);s<<"{\"stamp\":"<<msg->time().sec()+msg->time().nsec()*1e-9<<",\"pairs\":[";bool first=true;
  for(int i=0;i<msg->contact_size();++i){const auto& c=msg->contact(i);if(c.collision1().find("hummingbird")==std::string::npos && c.collision2().find("hummingbird")==std::string::npos)continue;
   if(!first)s<<",";first=false;s<<"{\"a\":\""<<c.collision1()<<"\",\"b\":\""<<c.collision2()<<"\",\"points\":"<<c.position_size()<<",\"depths\":[";
   for(int j=0;j<c.depth_size();++j)s<<(j?",":"")<<c.depth(j);s<<"]}";
  }s<<"]}";std_msgs::String out;out.data=s.str();pub.publish(out);
 }
};GZ_REGISTER_WORLD_PLUGIN(FuelContactObserver)
}
