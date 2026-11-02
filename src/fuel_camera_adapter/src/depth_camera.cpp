// SPDX-License-Identifier: Apache-2.0
#include <gazebo/plugins/DepthCameraPlugin.hh>
#include <gazebo/rendering/rendering.hh>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/String.h>
#include <cstring>
#include <sstream>
#include <limits>
#include <cmath>
namespace gazebo {
class FuelDepthCamera : public DepthCameraPlugin {
  std::unique_ptr<ros::NodeHandle> node;
  ros::Publisher images, raw, info, evidence;
  std::string frame;
  bool hidden=false;
 public:
  void Load(sensors::SensorPtr sensor,sdf::ElementPtr sdf) override {
    if(!ros::isInitialized()) { gzthrow("ROS must be initialized before camera"); }
    DepthCameraPlugin::Load(sensor,sdf);
    node.reset(new ros::NodeHandle("/hummingbird/vi_sensor/camera_depth"));
    frame=sdf->Get<std::string>("frameName");
    images=node->advertise<sensor_msgs::Image>("depth/disparity",2);
    raw=node->advertise<sensor_msgs::Image>("depth/native",2);
    info=node->advertise<sensor_msgs::CameraInfo>("depth/camera_info",2);
    evidence=node->advertise<std_msgs::String>("projection_evidence",1,true);
    // Apply to the actual OGRE camera, not just metadata. The same values are in SDF.
    auto intr=sdf;
    depthCamera->UpdateCameraIntrinsics(intr->Get<double>("fx"),intr->Get<double>("fy"),intr->Get<double>("cx"),intr->Get<double>("cy"),intr->Get<double>("s"));
    parentSensor->SetActive(true);
  }
  void OnNewDepthFrame(const float *data,unsigned int w,unsigned int h,unsigned int,const std::string&) override {
    if(!node) return;
    // Only this model's render visuals are excluded, never environment geometry or collisions.
    if(!hidden) { auto v=depthCamera->GetScene()->GetVisual("hummingbird"); if(v) { v->SetVisible(false,true); hidden=true; } }
    auto t=parentSensor->LastMeasurementTime();
    sensor_msgs::Image msg; msg.header.stamp=ros::Time(t.sec,t.nsec);msg.header.frame_id=frame;
    msg.width=w;msg.height=h;msg.encoding="32FC1";msg.step=4*w;msg.is_bigendian=false;msg.data.resize(4*w*h);
    std::memcpy(msg.data.data(),data,4*w*h);
    if(raw.getNumSubscribers())raw.publish(msg);
    float* out=reinterpret_cast<float*>(msg.data.data());
    for(size_t i=0;i<w*h;++i) if(!(data[i] > .1f && data[i] < 5.f))out[i]=std::numeric_limits<float>::quiet_NaN();
    // This cutoff is exactly the previous OpenNI 32FC1 policy; no near-pixel deletion.
    images.publish(msg);
    auto p=depthCamera->ProjectionMatrix();
    sensor_msgs::CameraInfo ci;ci.header=msg.header;ci.width=w;ci.height=h;ci.distortion_model="plumb_bob";ci.D.assign(5,0.0);
    double fx=p(0,0)*w/2.0,fy=p(1,1)*h/2.0,cx=(1-p(0,2))*w/2.0,cy=(1+p(1,2))*h/2.0;
    ci.K={{fx,0,cx,0,fy,cy,0,0,1}};ci.R={{1,0,0,0,1,0,0,0,1}};ci.P={{fx,0,cx,0,0,fy,cy,0,0,0,1,0}};info.publish(ci);
    std::ostringstream s;s.precision(17);s<<"{\"stamp\":"<<msg.header.stamp.toSec()<<",\"self_visual_hidden\":"<<(hidden?"true":"false")<<",\"projection\":[";
    for(int i=0;i<4;++i)for(int j=0;j<4;++j)s<<(i||j?",":"")<<p(i,j);
    auto pose=depthCamera->WorldPose();s<<"],\"world_pose\":["<<pose.Pos().X()<<","<<pose.Pos().Y()<<","<<pose.Pos().Z()<<","<<pose.Rot().Roll()<<","<<pose.Rot().Pitch()<<","<<pose.Rot().Yaw()<<"]}";
    std_msgs::String e;e.data=s.str();evidence.publish(e);
  }
};GZ_REGISTER_SENSOR_PLUGIN(FuelDepthCamera)
}
