#include <plan_env/regression_evidence.h>
#include <plan_env/sdf_map.h>
#include <boost/property_tree/json_parser.hpp>
#include <fstream>
#include <thread>
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <set>
#include <cstdlib>
#include <unistd.h>
namespace fuel_regression {
namespace {
struct Job{std::string event;J data;std::vector<double> bin;int cols;};
struct Writer{
 std::string dir;std::mutex m;std::condition_variable cv;std::deque<Job> q;bool stop=false;uint64_t dropped=0,index=0;std::thread thread;
 Writer(){const char* p=getenv("FUEL_REGRESSION_TRACE_DIR");if(p)dir=p;if(!dir.empty())thread=std::thread([this](){run();});}
 ~Writer(){if(thread.joinable()){{std::lock_guard<std::mutex> l(m);stop=true;}cv.notify_all();thread.join();}}
 void run(){std::ofstream f(dir+"/trace_"+std::to_string(getpid())+".jsonl");
  for(;;){Job j;{std::unique_lock<std::mutex> l(m);cv.wait(l,[&]{return stop||!q.empty();});if(q.empty()&&stop)break;j=std::move(q.front());q.pop_front();}
   if(!j.bin.empty()){std::string name="array_"+std::to_string(getpid())+"_"+std::to_string(++index)+".f64";std::ofstream b(dir+"/"+name,std::ios::binary);b.write(reinterpret_cast<const char*>(j.bin.data()),j.bin.size()*sizeof(double));b.close();j.data.put("binary",name);j.data.put("columns",j.cols);j.data.put("rows",j.bin.size()/j.cols);}
   j.data.put("event",j.event);boost::property_tree::write_json(f,j.data,false);f.flush();
  }}
};
Writer& writer(){static Writer w;return w;}
bool flags[4]={false,false,false,false};uint64_t tx=0;std::set<std::string> seen;
int fi(char c){return c=='Y'?0:c=='T'?1:c=='Q'?2:3;}
}
double steady(){return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();}
void configure(ros::NodeHandle& nh){for(char c:std::string("YTQG"))nh.param(std::string("regression/historical_")+c,flags[fi(c)],false);}
bool flag(char c){return flags[fi(c)];}
bool active(){const char* p=getenv("FUEL_REGRESSION_TRACE_DIR");return p&&*p;}
uint64_t nextTransaction(){return ++tx;}uint64_t transaction(){return tx;}
bool once(const std::string& k){return seen.insert(k).second;}
J vector(const Eigen::VectorXd& v){J a;for(int i=0;i<v.size();++i){J x;x.put("",v[i]);a.push_back({"",x});}return a;}
J vectors(const std::vector<Eigen::Vector3d>& v){J a;for(const auto& x:v)a.push_back({"",vector(x)});return a;}
void binary(const std::string& event,J d,std::vector<double> b,int cols){
 if(!active())return;d.put("stamp",ros::Time::now().toSec());d.put("steady",steady());d.put("transaction",tx);d.put("node",ros::this_node::getName());
 auto& w=writer();{std::lock_guard<std::mutex> l(w.m);if(w.q.size()>=256){++w.dropped;return;}d.put("writer_dropped",w.dropped);w.q.push_back(Job{event,std::move(d),std::move(b),cols});}w.cv.notify_one();
}
void emit(const std::string& e,J d){binary(e,std::move(d),{},1);}
}
namespace fast_planner {
void SDFMap::regressionSnapshot(const std::string& cause,const Eigen::Vector3d& root,const Eigen::Vector3d& goal){
 if(!fuel_regression::active())return;const double start=fuel_regression::steady();
 Eigen::Vector3d low=root.cwiseMin(goal)-Eigen::Vector3d::Constant(1.0),high=root.cwiseMax(goal)+Eigen::Vector3d::Constant(1.0);
 bool partial=(high-low).maxCoeff()>9.;if(partial){low=root-Eigen::Vector3d::Constant(1.5);high=root+Eigen::Vector3d::Constant(1.5);}
 Eigen::Vector3i a,b;posToIndex(low,a);posToIndex(high,b);boundIndex(a);boundIndex(b);
 auto gen=regression_generation_,esdf=regression_esdf_generation_;std::vector<double> data;data.reserve((b-a+Eigen::Vector3i::Ones()).prod()*8);
 for(int x=a.x();x<=b.x();++x)for(int y=a.y();y<=b.y();++y)for(int z=a.z();z<=b.z();++z){
  Eigen::Vector3i id(x,y,z);int adr=toAddress(id);data.insert(data.end(),{double(x),double(y),double(z),md_->occupancy_buffer_[adr],double(getOccupancy(id)),double(md_->occupancy_buffer_inflate_[adr]),md_->distance_buffer_[adr],double((id.array()>=md_->local_bound_min_.array()).all()&&(id.array()<=md_->local_bound_max_.array()).all())});
 }
 fuel_regression::J d;d.put("cause",cause);d.put("occupancy_generation",gen);d.put("esdf_generation",esdf);d.put("single_ros_spin_thread",true);d.put("generation_unchanged",gen==regression_generation_);d.put("partial_local_snapshot",true);d.put("far_goal_outside_local_choice",partial);d.put("resolution",mp_->resolution_);
 d.add_child("origin",fuel_regression::vector(mp_->map_origin_));d.add_child("map_size",fuel_regression::vector(mp_->map_size_));d.add_child("box_min",fuel_regression::vector(mp_->box_mind_));d.add_child("box_max",fuel_regression::vector(mp_->box_maxd_));d.add_child("index_min",fuel_regression::vector(a.cast<double>()));d.add_child("index_max",fuel_regression::vector(b.cast<double>()));
 d.add_child("root",fuel_regression::vector(root));d.add_child("goal",fuel_regression::vector(goal));d.put("capture_wall_seconds",fuel_regression::steady()-start);
 fuel_regression::binary("map_snapshot",d,std::move(data),8);
}
}
