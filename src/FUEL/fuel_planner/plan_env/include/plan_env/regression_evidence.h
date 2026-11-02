#pragma once
#ifdef inf
#pragma push_macro("inf")
#undef inf
#define FUEL_REGRESSION_RESTORE_INF
#endif
#include <boost/property_tree/ptree.hpp>
#include <Eigen/Eigen>
#include <ros/ros.h>
#include <functional>
#include <cmath>
#include <algorithm>
#include <vector>
namespace fast_planner { class SDFMap; }
namespace fuel_regression {
using J=boost::property_tree::ptree;
void configure(ros::NodeHandle& nh);
bool flag(char name);
bool active();
uint64_t nextTransaction();
uint64_t transaction();
void emit(const std::string& event,J data);
void binary(const std::string& event,J data,std::vector<double> values,int columns);
bool once(const std::string& key);
J vector(const Eigen::VectorXd& v);
template<class T> J eigen(const Eigen::MatrixBase<T>& m) {
 J rows;for(int i=0;i<m.rows();++i){J row;for(int j=0;j<m.cols();++j){J v;v.put("",m(i,j));row.push_back({"",v});}rows.push_back({"",row});}return rows;
}
template<class T> J array(const std::vector<T>& v){J a;for(const auto& n:v){J x;x.put("",n);a.push_back({"",x});}return a;}
J vectors(const std::vector<Eigen::Vector3d>& v);
struct Scope{std::function<void()> f;~Scope(){if(active())f();}};
inline double yawTimeBound(double start,double end,double yd,bool historical){
 const double diff=std::fabs(end-start);
 return historical?std::fabs(std::atan2(std::sin(end-start),std::cos(end-start)))/yd:std::min(diff,2*M_PI-diff)/yd;
}
double steady();
}

#ifdef FUEL_REGRESSION_RESTORE_INF
#pragma pop_macro("inf")
#undef FUEL_REGRESSION_RESTORE_INF
#endif
