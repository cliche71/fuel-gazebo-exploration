#pragma once
#include <bspline/non_uniform_bspline.h>
#include <Eigen/Eigen>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <vector>

namespace fast_planner {
struct ExecutionQuery { int inflated=0, occupancy=0; double distance=0.; bool inside=true; };
struct ExecutionPrefix {
  bool safe=false, kinematic=false;
  double checked_speed_upper_bound=0., checked_acceleration_peak=0.;
  double first_unsafe=0., last_safe=0., checked_end=0., minimum_distance=1e9;
  int samples=0;
  const char* reason="invalid_spline";
  Eigen::Vector3d point=Eigen::Vector3d::Zero();
};

// A quadratic velocity piece is represented by its three Bezier control vectors.
// Accepted leaf convex hulls bound the continuous vector norm; subdivision avoids
// rejecting a safe piece merely because a coarse hull crosses the interface limit.
inline bool velocityPieceWithin(const Eigen::Vector3d& b0,const Eigen::Vector3d& b1,
    const Eigen::Vector3d& b2,double lo,double hi,double limit,int depth,
    double& bad_phase,double& certified_bound,bool& unresolved) {
  const double upper=std::max({b0.norm(),b1.norm(),b2.norm()});
  if(upper<=limit){certified_bound=std::max(certified_bound,upper);return true;}
  const Eigen::Vector3d left=(b0+b1)*.5,right=(b1+b2)*.5,mid=(left+right)*.5;
  if(b0.norm()>limit){bad_phase=lo;certified_bound=std::max(certified_bound,upper);return false;}
  if(b2.norm()>limit){bad_phase=hi;certified_bound=std::max(certified_bound,upper);return false;}
  if(mid.norm()>limit){bad_phase=(lo+hi)*.5;certified_bound=std::max(certified_bound,upper);return false;}
  if(depth>=20){bad_phase=(lo+hi)*.5;unresolved=true;certified_bound=std::max(certified_bound,upper);return false;}
  return velocityPieceWithin(b0,left,mid,lo,(lo+hi)*.5,limit,depth+1,bad_phase,certified_bound,unresolved)&&
         velocityPieceWithin(mid,right,b2,(lo+hi)*.5,hi,limit,depth+1,bad_phase,certified_bound,unresolved);
}
inline ExecutionPrefix inspectExecutionPrefix(NonUniformBspline& trajectory, double begin,
    double horizon, double resolution, double margin,
    const std::function<ExecutionQuery(const Eigen::Vector3d&)>& query,
    double speed_limit=0.,double acceleration_limit=0.) {
  ExecutionPrefix r;
  auto q=trajectory.getControlPoint();auto knots=trajectory.getKnot();
  if(q.rows()<6||q.cols()!=3||!q.allFinite()||knots.size()!=q.rows()+4||
     !knots.allFinite()||!std::isfinite(begin)||!std::isfinite(horizon)||horizon<0.||
     !std::isfinite(resolution)||resolution<=0.||!std::isfinite(margin)||margin<0.)return r;
  for(int i=1;i<knots.size();++i)if(knots[i]<knots[i-1])return r;
  double duration=trajectory.getTimeSum();
  if(!std::isfinite(duration)||duration<=0.||begin<0.||begin>duration+1e-8)return r;
  begin=std::min(begin,duration);double end=std::min(duration,begin+horizon);
  auto velocity=trajectory.getDerivative();auto v=velocity.getControlPoint();
  if(!v.allFinite())return r;

  if(speed_limit>0.||acceleration_limit>0.){
    auto acceleration=velocity.getDerivative();
    std::vector<double> cuts={begin,end};
    for(int k=3;k<knots.size()-3;++k){
      double t=knots[k]-knots[3];if(t>begin+1e-10&&t<end-1e-10)cuts.push_back(t);
    }
    std::sort(cuts.begin(),cuts.end());cuts.erase(std::unique(cuts.begin(),cuts.end()),cuts.end());
    for(size_t k=0;k+1<cuts.size();++k){
      double lo=cuts[k],hi=cuts[k+1];if(hi-lo<1e-12)continue;
      Eigen::Vector3d a0=acceleration.evaluateDeBoorT(lo),a1=acceleration.evaluateDeBoorT(hi);
      double peak=std::max(a0.norm(),a1.norm());r.checked_acceleration_peak=std::max(r.checked_acceleration_peak,peak);
      if(acceleration_limit>0.&&peak>acceleration_limit){
        r.reason="command_acceleration_limit";r.kinematic=true;
        r.first_unsafe=a0.norm()>acceleration_limit?lo:hi;r.checked_end=r.first_unsafe;
        r.point=trajectory.evaluateDeBoorT(r.first_unsafe);return r;
      }
      if(speed_limit>0.){
        Eigen::Vector3d v0=velocity.evaluateDeBoorT(lo),v1=velocity.evaluateDeBoorT(hi),
                        vm=velocity.evaluateDeBoorT((lo+hi)*.5);
        Eigen::Vector3d b1=2.*vm-.5*(v0+v1);double bad=lo;bool unresolved=false;
        if(!velocityPieceWithin(v0,b1,v1,lo,hi,speed_limit,0,bad,r.checked_speed_upper_bound,unresolved)){
          r.reason=unresolved?"command_speed_bound_unresolved":"command_speed_limit";r.kinematic=true;
          r.first_unsafe=bad;r.checked_end=bad;r.point=trajectory.evaluateDeBoorT(bad);return r;
        }
      }
    }
  }

  double bound=v.rowwise().norm().maxCoeff();
  if(!std::isfinite(bound)){r.reason="nonfinite_velocity_bound";return r;}
  double step=std::min(.01,resolution*.25/std::max(bound,1e-6));
  double required=std::ceil((end-begin)/step);
  if(!std::isfinite(required)||required>50000){r.reason="sample_budget_exceeded";return r;}
  int count=std::max(1,int(required));
  r.last_safe=begin;r.checked_end=begin;
  for(int i=0;i<=count;++i){
    double phase=begin+(end-begin)*double(i)/count;
    Eigen::Vector3d p=trajectory.evaluateDeBoorT(phase);
    if(!p.allFinite()){r.reason="nonfinite_position";return r;}
    ExecutionQuery e=query(p);++r.samples;r.point=p;r.first_unsafe=phase;r.checked_end=phase;
    if(!e.inside){r.reason="outside_map";return r;}
    if(e.occupancy==-1){r.reason="unknown";return r;}
    if(e.inflated==1){r.reason="inflated";return r;}
    if(!std::isfinite(e.distance)){r.reason="nonfinite_distance";return r;}
    r.minimum_distance=std::min(r.minimum_distance,e.distance);
    if(e.distance+1e-10<margin){r.reason="tracking_margin";return r;}
    r.last_safe=phase;
  }
  r.safe=true;r.reason="checked_prefix";return r;
}
inline double stopBeforeUnsafe(double now_phase,double last_safe,double guard_time) {
  return std::max(now_phase,last_safe-std::max(0.,guard_time));
}
// ROS independent scheduling logic; the same implementation is used by traj_server and checks.
class PendingSafetyStop {
  std::map<uint32_t,double> pending_;
 public:
  enum Result { Invalid, Stale, Queued, Tightened, Unchanged };
  Result request(uint32_t id,double stamp,uint32_t current_id,double start,
                 double now,double& effective) {
    if(id==0||!std::isfinite(stamp)||stamp<=0.||!std::isfinite(now))return Invalid;
    if(id<current_id)return Stale;
    if(id>current_id){
      auto it=pending_.find(id);
      if(it==pending_.end())pending_[id]=stamp;else it->second=std::min(it->second,stamp);
      if(pending_.size()>16)pending_.erase(std::prev(pending_.end()));
      return Queued;
    }
    if(!std::isfinite(start)||!std::isfinite(effective)||effective<0.)return Invalid;
    // A late deadline holds the current point, never jumps backwards along the curve.
    double phase=std::max(0.,now-start);
    double stop=std::max(phase,std::max(0.,stamp-start));
    double before=effective;effective=std::min(effective,stop);
    return effective<before?Tightened:Unchanged;
  }
  Result switched(uint32_t id,double start,double now,double& effective) {
    auto it=pending_.find(id);double stamp=it==pending_.end()?0.:it->second;
    pending_.erase(pending_.begin(),pending_.upper_bound(id));
    return stamp>0.?request(id,stamp,id,start,now,effective):Unchanged;
  }
};
}
