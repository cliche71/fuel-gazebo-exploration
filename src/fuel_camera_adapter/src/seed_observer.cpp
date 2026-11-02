#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Rand.hh>
#include <ros/ros.h>
#include <std_msgs/String.h>
namespace gazebo {class FuelSeedEvidence:public WorldPlugin{
 ros::NodeHandle nh;ros::Publisher pub;
 public:void Load(physics::WorldPtr world,sdf::ElementPtr) override{
 pub=nh.advertise<std_msgs::String>("/fuel_evidence/runtime",1,true);std_msgs::String m;
 m.data="{\"gazebo_seed\":"+std::to_string(ignition::math::Rand::Seed())+",\"world\":\""+world->Name()+"\"}";pub.publish(m);
 }};GZ_REGISTER_WORLD_PLUGIN(FuelSeedEvidence)}
