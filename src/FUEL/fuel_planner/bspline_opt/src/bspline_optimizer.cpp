#include "bspline_opt/bspline_optimizer.h"
#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>
#include <thread>

#include <nlopt.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <plan_env/regression_evidence.h>
// using namespace std;

namespace fast_planner {
const int BsplineOptimizer::SMOOTHNESS = (1 << 0);
const int BsplineOptimizer::DISTANCE = (1 << 1);
const int BsplineOptimizer::FEASIBILITY = (1 << 2);
const int BsplineOptimizer::START = (1 << 3);
const int BsplineOptimizer::END = (1 << 4);
const int BsplineOptimizer::GUIDE = (1 << 5);
const int BsplineOptimizer::WAYPOINTS = (1 << 6);
const int BsplineOptimizer::VIEWCONS = (1 << 7);
const int BsplineOptimizer::MINTIME = (1 << 8);

const int BsplineOptimizer::GUIDE_PHASE = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::GUIDE |
    BsplineOptimizer::START | BsplineOptimizer::END;
const int BsplineOptimizer::NORMAL_PHASE = BsplineOptimizer::SMOOTHNESS | BsplineOptimizer::DISTANCE |
    BsplineOptimizer::FEASIBILITY | BsplineOptimizer::START | BsplineOptimizer::END;

namespace {
// Each cubic uniform span has quadratic velocity and linear acceleration. These
// extrema and their control-point gradients cover the complete continuous span.
struct YawSpanExtrema {
  double vmax, vmin, a0, a1;
  Eigen::Vector4d gvmax, gvmin, ga0, ga1;
};

Eigen::Vector4d yawRateBasis(double s, double dt) {
  const double s2 = s * s;
  return Eigen::Vector4d(-0.5 * s2 + s - 0.5, 1.5 * s2 - 2.0 * s,
                        -1.5 * s2 + s + 0.5, 0.5 * s2) / dt;
}

YawSpanExtrema yawSpanExtrema(const Eigen::Vector4d& q, double dt) {
  YawSpanExtrema out;
  out.gvmax = out.gvmin = yawRateBasis(0.0, dt);
  out.vmax = out.vmin = out.gvmax.dot(q);
  const auto consider = [&out, &q, dt](double s) {
    const Eigen::Vector4d basis = yawRateBasis(s, dt);
    const double value = basis.dot(q);
    if (value > out.vmax) { out.vmax = value; out.gvmax = basis; }
    if (value < out.vmin) { out.vmin = value; out.gvmin = basis; }
  };
  consider(1.0);
  const double A = (-q[0] + 3.0 * q[1] - 3.0 * q[2] + q[3]) / (2.0 * dt);
  const double B = (q[0] - 2.0 * q[1] + q[2]) / dt;
  // An interior stationary point exists only for a nonzero quadratic term.
  if (A != 0.0) {
    const double s = -B / (2.0 * A);
    if (std::isfinite(s) && s > 0.0 && s < 1.0) consider(s);
  }
  out.ga0 = Eigen::Vector4d(1.0, -2.0, 1.0, 0.0) / (dt * dt);
  out.ga1 = Eigen::Vector4d(0.0, 1.0, -2.0, 1.0) / (dt * dt);
  out.a0 = out.ga0.dot(q);
  out.a1 = out.ga1.dot(q);
  return out;
}

struct YawConstraintData {
  double dt, rate_limit, acc_limit;
};

void yawMotionConstraints(unsigned m, double* result, unsigned n, const double* x,
                          double* grad, void* raw_data) {
  const auto& data = *static_cast<YawConstraintData*>(raw_data);
  if (grad) std::fill(grad, grad + size_t(m) * n, 0.0);
  for (unsigned i = 0; i + 3 < n; ++i) {
    const Eigen::Vector4d q(x[i], x[i + 1], x[i + 2], x[i + 3]);
    const YawSpanExtrema ext = yawSpanExtrema(q, data.dt);
    const unsigned k = 6 * i;
    result[k] = ext.vmax - data.rate_limit;
    result[k + 1] = -ext.vmin - data.rate_limit;
    result[k + 2] = ext.a0 - data.acc_limit;
    result[k + 3] = -ext.a0 - data.acc_limit;
    result[k + 4] = ext.a1 - data.acc_limit;
    result[k + 5] = -ext.a1 - data.acc_limit;
    // The first three control points are fixed. Their initial acceleration
    // is already checked before solving. Repeating it as an active zero-gradient inequality
    // can make SLSQP fail on harmless floating-point boundary residuals.
    if (i == 0) result[k + 2] = result[k + 3] = -1.0;
    if (grad) {
      for (unsigned j = 0; j < 4; ++j) {
        grad[size_t(k) * n + i + j] = ext.gvmax[j];
        grad[size_t(k + 1) * n + i + j] = -ext.gvmin[j];
        grad[size_t(k + 2) * n + i + j] = i == 0 ? 0.0 : ext.ga0[j];
        grad[size_t(k + 3) * n + i + j] = i == 0 ? 0.0 : -ext.ga0[j];
        grad[size_t(k + 4) * n + i + j] = ext.ga1[j];
        grad[size_t(k + 5) * n + i + j] = -ext.ga1[j];
      }
    }
  }
}
}  // namespace

bool BsplineOptimizer::checkYawMotion(const Eigen::MatrixXd& points, double dt,
                                      const Eigen::Vector3d& start_yaw, double rate_limit,
                                      double acc_limit, double* peak_rate, double* peak_acc,
                                      double* start_error) {
  const double infinity = std::numeric_limits<double>::infinity();
  if (peak_rate) *peak_rate = infinity;
  if (peak_acc) *peak_acc = infinity;
  if (start_error) *start_error = infinity;
  if (points.cols() != 1 || points.rows() < 4 || !points.allFinite() ||
      !start_yaw.allFinite() || !std::isfinite(dt) || dt <= 1e-6 ||
      !std::isfinite(rate_limit) || rate_limit <= 0.0 ||
      !std::isfinite(acc_limit) || acc_limit <= 0.0) return false;

  const double yaw0 = (points(0, 0) + 4.0 * points(1, 0) + points(2, 0)) / 6.0;
  const double rate0 = (points(2, 0) - points(0, 0)) / (2.0 * dt);
  const double acc0 = (points(0, 0) - 2.0 * points(1, 0) + points(2, 0)) / (dt * dt);
  const double error = (Eigen::Vector3d(yaw0, rate0, acc0) - start_yaw).cwiseAbs().maxCoeff();
  double max_rate = 0.0, max_acc = 0.0;
  for (int i = 0; i + 3 < points.rows(); ++i) {
    const YawSpanExtrema ext = yawSpanExtrema(points.block<4, 1>(i, 0), dt);
    max_rate = std::max(max_rate, std::max(std::fabs(ext.vmax), std::fabs(ext.vmin)));
    max_acc = std::max(max_acc, std::max(std::fabs(ext.a0), std::fabs(ext.a1)));
  }
  if (peak_rate) *peak_rate = max_rate;
  if (peak_acc) *peak_acc = max_acc;
  if (start_error) *start_error = error;
  return std::isfinite(max_rate) && std::isfinite(max_acc) && std::isfinite(error) &&
         error <= 1e-3 && max_rate <= rate_limit + 1e-7 && max_acc <= acc_limit + 1e-7;
}

bool BsplineOptimizer::refineYawWithLimits(Eigen::MatrixXd& points, double dt,
                                           double rate_limit, double acc_limit) {
  // This entry point is opt-in and follows the normal yaw optimize() call. The
  // original objective, waypoints, weights, and soft terminal states stay intact.
  if (dim_ != 1 || order_ != 3 || optimize_time_ || points.cols() != 1 ||
      points.rows() != point_num_ || points.rows() < 4 || !points.allFinite() ||
      start_state_.size() < 3 || !std::isfinite(dt) || dt <= 1e-6 ||
      !std::isfinite(rate_limit) || rate_limit <= 0.0 ||
      !std::isfinite(acc_limit) || acc_limit <= 0.0) return false;
  const Eigen::Vector3d start(start_state_[0][0], start_state_[1][0], start_state_[2][0]);
  if (!start.allFinite() || std::fabs(start[1]) > rate_limit + 1e-7 ||
      std::fabs(start[2]) > acc_limit + 1e-7) return false;
  if (checkYawMotion(points, dt, start, rate_limit, acc_limit)) return true;
  if (!std::isfinite(pt_dist_) || pt_dist_ <= 1e-12 || variable_num_ != point_num_ ||
      (cost_function_ & MINTIME)) return false;

  Eigen::Matrix3d states2pts;
  states2pts << 1.0, -dt, dt * dt / 3.0,
                1.0, 0.0, -dt * dt / 6.0,
                1.0, dt, dt * dt / 3.0;
  const Eigen::Vector3d fixed = states2pts * start;
  std::vector<double> q(point_num_), lower(point_num_, -std::numeric_limits<double>::infinity()),
      upper(point_num_, std::numeric_limits<double>::infinity());
  for (int i = 0; i < point_num_; ++i) q[i] = points(i, 0);
  for (int i = 0; i < 3; ++i) q[i] = lower[i] = upper[i] = fixed[i];
  knot_span_ = dt;

  // Start from a feasible braking continuation when one exists. Retaining only
  // feasible objective iterates lets a solver time/roundoff stop return a safe
  // solution instead of an unchecked or infeasible best-cost iterate.
  std::vector<double> seed = q;
  bool seed_ok = true;
  for (int i = 0; i + 3 < point_num_ && seed_ok; ++i) {
    const double v0 = (seed[i + 2] - seed[i]) / (2.0 * dt);
    const double a0 = (seed[i] - 2.0 * seed[i + 1] + seed[i + 2]) / (dt * dt);
    const double choices[] = {0.0, -acc_limit, acc_limit, -a0,
      -2.0 * v0 / dt - a0,
      2.0 * (rate_limit - v0) / dt - a0,
      2.0 * (-rate_limit - v0) / dt - a0};
    double best_score = std::numeric_limits<double>::infinity(), best_point = 0.0;
    for (double a1 : choices) {
      a1 = std::max(-acc_limit, std::min(acc_limit, a1));
      const double next = a1 * dt * dt - seed[i + 1] + 2.0 * seed[i + 2];
      const YawSpanExtrema ext = yawSpanExtrema(
          Eigen::Vector4d(seed[i], seed[i + 1], seed[i + 2], next), dt);
      if (std::max(std::fabs(ext.vmax), std::fabs(ext.vmin)) > rate_limit + 1e-8 ||
          std::max(std::fabs(ext.a0), std::fabs(ext.a1)) > acc_limit + 1e-7) continue;
      const double vend = (next - seed[i + 1]) / (2.0 * dt);
      const double score = vend * vend + 0.1 * dt * dt * a1 * a1;
      if (score < best_score) { best_score = score; best_point = next; }
    }
    seed_ok = std::isfinite(best_score);
    if (seed_ok) seed[i + 3] = best_point;
  }
  if (seed_ok) q = seed;

  struct FeasibleObjective {
    BsplineOptimizer* optimizer;
    Eigen::Vector3d start, fixed;
    double dt, rate, acc, best_cost;
    std::vector<double> best;
  } objective{this, start, fixed, dt, rate_limit, acc_limit,
              std::numeric_limits<double>::infinity(), {}};
  int regression_refine_code=0;std::string regression_refine_exception;
  fuel_regression::Scope regression_refine_scope{[&]{
    fuel_regression::J d;d.put("dt",dt);d.put("rate_limit",rate_limit);d.put("acc_limit",acc_limit);d.put("return_code",regression_refine_code);d.put("exception",regression_refine_exception);d.add_child("returned",fuel_regression::array(q));d.add_child("feasible_best",fuel_regression::array(objective.best));d.add_child("production_best_variable",fuel_regression::array(best_variable_));if(std::isfinite(objective.best_cost))d.put("best_cost",objective.best_cost);fuel_regression::emit("yaw_refine",d);
  }};
  const auto bounded_cost = [](const std::vector<double>& x, std::vector<double>& gradient,
                                void* raw) -> double {
    auto& context = *static_cast<FeasibleObjective*>(raw);
    const double value = BsplineOptimizer::costFunction(x, gradient, context.optimizer);
    Eigen::MatrixXd candidate(x.size(), 1);
    for (size_t i = 0; i < x.size(); ++i) candidate(i, 0) = x[i];
    if (std::isfinite(value) && value < context.best_cost &&
        (candidate.block<3, 1>(0, 0) - context.fixed).cwiseAbs().maxCoeff() < 1e-9 &&
        BsplineOptimizer::checkYawMotion(candidate, context.dt, context.start,
                                         context.rate, context.acc)) {
      context.best_cost = value;
      context.best = x;
    }
    return value;
  };
  std::vector<double> initial_gradient;
  bounded_cost(q, initial_gradient, &objective);
  YawConstraintData data{dt, rate_limit, acc_limit};
  try {
    nlopt::opt solver(nlopt::LD_SLSQP, point_num_);
    // Use the existing cost function, retaining only independently verified
    // feasible iterates; the original best_variable_ can be infeasible.
    solver.set_min_objective(bounded_cost, &objective);
    solver.set_lower_bounds(lower);
    solver.set_upper_bounds(upper);
    solver.add_inequality_mconstraint(yawMotionConstraints, &data,
                                     std::vector<double>(6 * (point_num_ - 3), 1e-9));
    solver.set_maxeval(300);
    solver.set_maxtime(0.02);
    solver.set_xtol_rel(1e-8);
    solver.set_ftol_rel(1e-9);
    double final_cost = std::numeric_limits<double>::infinity();
    regression_refine_code=int(solver.optimize(q, final_cost));
  } catch (const std::exception& e) {
    regression_refine_exception=e.what();
    // Roundoff or a time limit can still leave a valid returned iterate. It is
    // never accepted unless all continuous-span and boundary checks below pass.
  }

  std::vector<double> final_gradient;
  bounded_cost(q, final_gradient, &objective);
  if (objective.best.empty()) return false;
  Eigen::MatrixXd candidate(point_num_, 1);
  for (int i = 0; i < point_num_; ++i) candidate(i, 0) = objective.best[i];
  if (!candidate.allFinite() ||
      (candidate.block<3, 1>(0, 0) - fixed).cwiseAbs().maxCoeff() > 1e-9 ||
      !checkYawMotion(candidate, dt, start, rate_limit, acc_limit)) return false;
  points = candidate;
  control_points_ = candidate;
  return true;
}

void BsplineOptimizer::setParam(ros::NodeHandle& nh) {
  nh.param("optimization/ld_smooth", ld_smooth_, -1.0);
  nh.param("optimization/ld_dist", ld_dist_, -1.0);
  nh.param("optimization/ld_feasi", ld_feasi_, -1.0);
  nh.param("optimization/ld_start", ld_start_, -1.0);
  nh.param("optimization/ld_end", ld_end_, -1.0);
  nh.param("optimization/ld_guide", ld_guide_, -1.0);
  nh.param("optimization/ld_waypt", ld_waypt_, -1.0);
  nh.param("optimization/ld_view", ld_view_, -1.0);
  nh.param("optimization/ld_time", ld_time_, -1.0);

  nh.param("optimization/dist0", dist0_, -1.0);
  nh.param("optimization/max_vel", max_vel_, -1.0);
  nh.param("optimization/max_acc", max_acc_, -1.0);
  nh.param("optimization/dlmin", dlmin_, -1.0);
  nh.param("optimization/wnl", wnl_, -1.0);

  nh.param("optimization/max_iteration_num1", max_iteration_num_[0], -1);
  nh.param("optimization/max_iteration_num2", max_iteration_num_[1], -1);
  nh.param("optimization/max_iteration_num3", max_iteration_num_[2], -1);
  nh.param("optimization/max_iteration_num4", max_iteration_num_[3], -1);
  nh.param("optimization/max_iteration_time1", max_iteration_time_[0], -1.0);
  nh.param("optimization/max_iteration_time2", max_iteration_time_[1], -1.0);
  nh.param("optimization/max_iteration_time3", max_iteration_time_[2], -1.0);
  nh.param("optimization/max_iteration_time4", max_iteration_time_[3], -1.0);

  nh.param("optimization/algorithm1", algorithm1_, -1);
  nh.param("optimization/algorithm2", algorithm2_, -1);
  nh.param("manager/bspline_degree", bspline_degree_, 3);

  time_lb_ = -1;  // Not used by in most case
}

void BsplineOptimizer::setEnvironment(const EDTEnvironment::Ptr& env) {
  this->edt_environment_ = env;
  dynamic_ = false;
}

void BsplineOptimizer::setCostFunction(const int& cost_code) {
  cost_function_ = cost_code;

  // print optimized cost function
  string cost_str;
  if (cost_function_ & SMOOTHNESS) cost_str += "smooth |";
  if (cost_function_ & DISTANCE) cost_str += " dist  |";
  if (cost_function_ & FEASIBILITY) cost_str += " feasi |";
  if (cost_function_ & START) cost_str += " start |";
  if (cost_function_ & END) cost_str += " end   |";
  if (cost_function_ & GUIDE) cost_str += " guide |";
  if (cost_function_ & WAYPOINTS) cost_str += " waypt |";
  if (cost_function_ & VIEWCONS) cost_str += " view  |";
  if (cost_function_ & MINTIME) cost_str += " time  |";

  // ROS_INFO_STREAM("cost func: " << cost_str);
}

void BsplineOptimizer::setGuidePath(const vector<Eigen::Vector3d>& guide_pt) {
  guide_pts_ = guide_pt;
}

void BsplineOptimizer::setWaypoints(const vector<Eigen::Vector3d>& waypts,
                                    const vector<int>& waypt_idx) {
  waypoints_ = waypts;
  waypt_idx_ = waypt_idx;
}

void BsplineOptimizer::setViewConstraint(const ViewConstraint& vc) {
  view_cons_ = vc;
}

void BsplineOptimizer::enableDynamic(double time_start) {
  dynamic_ = true;
  start_time_ = time_start;
}

void BsplineOptimizer::setBoundaryStates(const vector<Eigen::Vector3d>& start,
                                         const vector<Eigen::Vector3d>& end) {
  start_state_ = start;
  end_state_ = end;
}

void BsplineOptimizer::setTimeLowerBound(const double& lb) {
  time_lb_ = lb;
}

void BsplineOptimizer::optimize(Eigen::MatrixXd& points, double& dt, const int& cost_function,
                                const int& max_num_id, const int& max_time_id) {
  if (start_state_.empty()) {
    ROS_ERROR("Initial state undefined!");
    return;
  }
  control_points_ = points;
  knot_span_ = dt;
  max_num_id_ = max_num_id;
  max_time_id_ = max_time_id;
  setCostFunction(cost_function);

  // Set necessary data and flag
  dim_ = control_points_.cols();
  if (dim_ == 1)
    order_ = 3;
  else
    order_ = bspline_degree_;
  point_num_ = control_points_.rows();
  optimize_time_ = cost_function_ & MINTIME;
  variable_num_ = optimize_time_ ? dim_ * point_num_ + 1 : dim_ * point_num_;
  if (variable_num_ <= 0) {
    ROS_ERROR("Empty varibale to optimization solver.");
    return;
  }

  pt_dist_ = 0.0;
  for (int i = 0; i < control_points_.rows() - 1; ++i) {
    pt_dist_ += (control_points_.row(i + 1) - control_points_.row(i)).norm();
  }
  pt_dist_ /= double(point_num_);

  iter_num_ = 0;
  min_cost_ = std::numeric_limits<double>::max();
  g_q_.resize(point_num_);
  g_smoothness_.resize(point_num_);
  g_distance_.resize(point_num_);
  g_feasibility_.resize(point_num_);
  g_start_.resize(point_num_);
  g_end_.resize(point_num_);
  g_guide_.resize(point_num_);
  g_waypoints_.resize(point_num_);
  g_view_.resize(point_num_);
  g_time_.resize(point_num_);

  comb_time = 0.0;

  optimize();

  points = control_points_;
  dt = knot_span_;
  start_state_.clear();
  time_lb_ = -1;
}

void BsplineOptimizer::optimize() {
  // Optimize all control points and maybe knot span dt
  // Use NLopt solver
  nlopt::opt opt(nlopt::algorithm(isQuadratic() ? algorithm1_ : algorithm2_), variable_num_);
  opt.set_min_objective(BsplineOptimizer::costFunction, this);
  opt.set_maxeval(max_iteration_num_[max_num_id_]);
  opt.set_maxtime(max_iteration_time_[max_time_id_]);
  opt.set_xtol_rel(1e-5);

  // Set axis aligned bounding box for optimization
  Eigen::Vector3d bmin, bmax;
  edt_environment_->sdf_map_->getBox(bmin, bmax);
  for (int k = 0; k < 3; ++k) {
    bmin[k] += 0.1;
    bmax[k] -= 0.1;
  }
  // Deprecated: does not optimize start and end control points
  // for (int i = order_; i < pt_num; ++i)
  // {
  //   if (!(cost_function_ & BOUNDARY) && i >= pt_num - order_)
  //     continue;
  //   for (int j = 0; j < dim_; j++)
  //   {
  //     double cij = control_points_(i, j);
  //     if (dim_ != 1)
  //       cij = max(min(cij, bmax[j % 3]), bmin[j % 3]);
  //     q[dim_ * (i - order_) + j] = cij;
  //   }
  // }

  vector<double> q(variable_num_);
  // Variables for control points
  for (int i = 0; i < point_num_; ++i)
    for (int j = 0; j < dim_; ++j) {
      double cij = control_points_(i, j);
      if (dim_ != 1) cij = max(min(cij, bmax[j % 3]), bmin[j % 3]);
      q[dim_ * i + j] = cij;
    }
  // Variables for knot span
  if (optimize_time_) q[variable_num_ - 1] = knot_span_;

  std::vector<double> regression_lower,regression_upper;
  if (dim_ != 1) {
    vector<double> lb(variable_num_), ub(variable_num_);
    const double bound = 10.0;
    for (int i = 0; i < 3 * point_num_; ++i) {
      lb[i] = q[i] - bound;
      ub[i] = q[i] + bound;
      lb[i] = max(lb[i], bmin[i % 3]);
      ub[i] = min(ub[i], bmax[i % 3]);
    }
    if (optimize_time_) {
      lb[variable_num_ - 1] = std::max(1e-6, std::min(0.05, knot_span_ * 0.1));
      ub[variable_num_ - 1] = 5.0;
    }
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);regression_lower=lb;regression_upper=ub;
  }

  best_variable_ = q;  // Finite fallback if NLopt stops before its first objective call.
  fuel_regression::J evidence;
  if(fuel_regression::active()){
   evidence.put("dimension",dim_);evidence.put("dt",knot_span_);evidence.put("cost_function",cost_function_);evidence.put("max_num_id",max_num_id_);evidence.put("max_time_id",max_time_id_);evidence.put("time_lb",time_lb_);evidence.put("pt_dist",pt_dist_);evidence.put("maxeval",max_iteration_num_[max_num_id_]);evidence.put("maxtime",max_iteration_time_[max_time_id_]);evidence.put("historical_G",fuel_regression::flag('G'));
   evidence.add_child("lower_bounds",fuel_regression::array(regression_lower));evidence.add_child("upper_bounds",fuel_regression::array(regression_upper));evidence.put("weights.smooth",ld_smooth_);evidence.put("weights.dist",ld_dist_);evidence.put("weights.feasi",ld_feasi_);evidence.put("weights.start",ld_start_);evidence.put("weights.end",ld_end_);evidence.put("weights.waypt",ld_waypt_);evidence.put("weights.time",ld_time_);
   evidence.add_child("initial_variables",fuel_regression::array(q));evidence.add_child("initial_control_points",fuel_regression::eigen(control_points_));evidence.add_child("start_states",fuel_regression::vectors(start_state_));evidence.add_child("end_states",fuel_regression::vectors(end_state_));evidence.add_child("waypoints",fuel_regression::vectors(waypoints_));evidence.add_child("waypoint_indices",fuel_regression::array(waypt_idx_));
  }
  auto t1 = ros::Time::now();int regression_return=0;std::string regression_exception;bool regression_normal=false;double final_cost=std::numeric_limits<double>::quiet_NaN();
  try {
    nlopt::result result = opt.optimize(q, final_cost);regression_return=int(result);regression_normal=true;
  } catch (std::exception& e) {
    regression_exception=e.what();cout << e.what() << endl;
  }
  if(fuel_regression::active()){
   evidence.put("return_code",regression_return);evidence.put("exception",regression_exception);evidence.put("returned_normally",regression_normal);if(regression_normal)evidence.put("returned_cost",final_cost);
   evidence.put("iterations",iter_num_);evidence.put("min_cost",min_cost_);evidence.add_child("returned_variables",fuel_regression::array(q));evidence.add_child("adopted_best_variables",fuel_regression::array(best_variable_));fuel_regression::emit("optimizer",evidence);
   if(dim_==3&&fuel_regression::once("position_optimizer_map"))edt_environment_->sdf_map_->regressionSnapshot("position_optimizer_input",control_points_.colwise().minCoeff().transpose(),control_points_.colwise().maxCoeff().transpose());
  }
  for (int i = 0; i < point_num_; ++i)
    for (int j = 0; j < dim_; ++j)
      control_points_(i, j) = best_variable_[dim_ * i + j];
  if (optimize_time_) knot_span_ = best_variable_[variable_num_ - 1];

  if (cost_function_ & MINTIME) {
    std::cout << "Iter num: " << iter_num_ << ", time: " << (ros::Time::now() - t1).toSec()
              << ", point num: " << point_num_ << ", comb time: " << comb_time << std::endl;
  }

  // Deprecated
  // for (int i = order_; i < control_points_.rows(); ++i)
  // {
  //   if (!(cost_function_ & BOUNDARY) && i >= pt_num - order_)
  //     continue;
  //   for (int j = 0; j < dim_; j++)
  //   {
  //     control_points_(i, j) = best_variable_[dim_ * (i - order_) + j];
  //   }
  // }

  // if (!(cost_function_ & GUIDE))
  //   ROS_INFO_STREAM("iter num: " << iter_num_);
}

void BsplineOptimizer::calcSmoothnessCost(const vector<Eigen::Vector3d>& q, const double& dt,
                                          double& cost, vector<Eigen::Vector3d>& gradient_q,
                                          double& gt) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);
  Eigen::Vector3d jerk, temp_j;

  for (int i = 0; i < q.size() - 3; i++) {
    // Test jerk cost
    Eigen::Vector3d ji = (q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i]) / pt_dist_;
    cost += ji.squaredNorm();
    temp_j = 2 * ji / pt_dist_;

    // /* evaluate jerk */
    // jerk = q[i + 3] - 3 * q[i + 2] + 3 * q[i + 1] - q[i];
    // cost += jerk.squaredNorm();
    // temp_j = 2.0 * jerk;
    // /* jerk gradient_q */

    gradient_q[i + 0] += -temp_j;
    gradient_q[i + 1] += 3.0 * temp_j;
    gradient_q[i + 2] += -3.0 * temp_j;
    gradient_q[i + 3] += temp_j;
    // if (optimize_time_)
    //   gt += -6 * ji.dot(ji) / dt;
  }
}

void BsplineOptimizer::calcDistanceCost(const vector<Eigen::Vector3d>& q, double& cost,
                                        vector<Eigen::Vector3d>& gradient_q) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);

  double dist;
  Eigen::Vector3d dist_grad, g_zero(0, 0, 0);
  for (int i = 0; i < q.size(); i++) {
    if (!dynamic_) {
      edt_environment_->evaluateEDTWithGrad(q[i], -1.0, dist, dist_grad);
      if (dist_grad.norm() > 1e-4) dist_grad.normalize();
    } else {
      double time = double(i + 2 - order_) * knot_span_ + start_time_;
      edt_environment_->evaluateEDTWithGrad(q[i], time, dist, dist_grad);
    }

    if (dist < dist0_) {
      cost += pow(dist - dist0_, 2);
      gradient_q[i] += 2.0 * (dist - dist0_) * dist_grad;
    }
  }
}

void BsplineOptimizer::calcFeasibilityCost(const vector<Eigen::Vector3d>& q, const double& dt,
                                           double& cost, vector<Eigen::Vector3d>& gradient_q,
                                           double& gt) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);
  gt = 0.0;

  // Abbreviation of params
  const double dt_inv = 1 / dt;
  const double dt_inv2 = dt_inv * dt_inv;

  for (int i = 0; i < q.size() - 1; ++i) {
    // Control point of velocity
    Eigen::Vector3d vi = (q[i + 1] - q[i]) * dt_inv;
    for (int k = 0; k < 3; ++k) {
      // Calculate cost for each axis
      double vd = fabs(vi[k]) - max_vel_;
      if (vd > 0.0) {
        cost += pow(vd, 2);
        double sign = vi[k] > 0 ? 1.0 : -1.0;
        double tmp = 2 * vd * sign * dt_inv;
        gradient_q[i][k] += -tmp;
        gradient_q[i + 1][k] += tmp;
        if (optimize_time_) gt += tmp * (-vi[k]);
      }
    }
  }

  // Acc feasibility cost
  for (int i = 0; i < q.size() - 2; ++i) {
    Eigen::Vector3d ai = (q[i + 2] - 2 * q[i + 1] + q[i]) * dt_inv2;
    for (int k = 0; k < 3; ++k) {
      double ad = fabs(ai[k]) - max_acc_;
      if (ad > 0.0) {
        cost += pow(ad, 2);
        double sign = ai[k] > 0 ? 1.0 : -1.0;
        double tmp = 2 * ad * sign * dt_inv2;
        gradient_q[i][k] += tmp;
        gradient_q[i + 1][k] += -2 * tmp;
        gradient_q[i + 2][k] += tmp;
        if (optimize_time_) gt += tmp * ai[k] * (-2) * dt;
      }
    }
  }
}

void BsplineOptimizer::calcStartCost(const vector<Eigen::Vector3d>& q, const double& dt, double& cost,
                                     vector<Eigen::Vector3d>& gradient_q, double& gt) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  // std::fill(gradient_q.begin(), gradient_q.end(), zero);
  for (int i = 0; i < 3; ++i)
    gradient_q[i] = zero;
  gt = 0.0;

  Eigen::Vector3d q1, q2, q3, dq;
  q1 = q[0];
  q2 = q[1];
  q3 = q[2];

  // Start position
  static const double w_pos = 10.0;
  dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - start_state_[0];
  cost += w_pos * dq.squaredNorm();
  gradient_q[0] += w_pos * 2 * dq * (1 / 6.0);
  gradient_q[1] += w_pos * 2 * dq * (4 / 6.0);
  gradient_q[2] += w_pos * 2 * dq * (1 / 6.0);

  // Start velocity
  dq = 1 / (2 * dt) * (q3 - q1) - start_state_[1];
  cost += dq.squaredNorm();
  gradient_q[0] += 2 * dq * (-1.0) / (2 * dt);
  gradient_q[2] += 2 * dq * 1.0 / (2 * dt);
  if (optimize_time_) gt += dq.dot(q3 - q1) / (-dt * dt);

  // Start acceleration
  dq = 1 / (dt * dt) * (q1 - 2 * q2 + q3) - start_state_[2];
  cost += dq.squaredNorm();
  gradient_q[0] += 2 * dq * 1.0 / (dt * dt);
  gradient_q[1] += 2 * dq * (-2.0) / (dt * dt);
  gradient_q[2] += 2 * dq * 1.0 / (dt * dt);
  if (optimize_time_) gt += (fuel_regression::flag('G')?4.0:1.0) * dq.dot(q1 - 2 * q2 + q3) / (-dt * dt * dt);
}

void BsplineOptimizer::calcEndCost(const vector<Eigen::Vector3d>& q, const double& dt, double& cost,
                                   vector<Eigen::Vector3d>& gradient_q, double& gt) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  // std::fill(gradient_q.begin(), gradient_q.end(), zero);
  for (int i = q.size() - 3; i < q.size(); ++i)
    gradient_q[i] = zero;
  gt = 0.0;

  Eigen::Vector3d q_3, q_2, q_1, dq;
  q_3 = q[q.size() - 3];
  q_2 = q[q.size() - 2];
  q_1 = q[q.size() - 1];

  // End position
  dq = 1 / 6.0 * (q_1 + 4 * q_2 + q_3) - end_state_[0];
  cost += dq.squaredNorm();
  gradient_q[q.size() - 1] += 2 * dq * (1 / 6.0);
  gradient_q[q.size() - 2] += 2 * dq * (4 / 6.0);
  gradient_q[q.size() - 3] += 2 * dq * (1 / 6.0);

  if (end_state_.size() >= 2) {
    // End velocity
    dq = 1 / (2 * dt) * (q_1 - q_3) - end_state_[1];
    cost += dq.squaredNorm();
    gradient_q[q.size() - 1] += 2 * dq * 1.0 / (2 * dt);
    gradient_q[q.size() - 3] += 2 * dq * (-1.0) / (2 * dt);
    if (optimize_time_) gt += dq.dot(q_1 - q_3) / (-dt * dt);
  }
  if (end_state_.size() == 3) {
    // End acceleration
    dq = 1 / (dt * dt) * (q_1 - 2 * q_2 + q_3) - end_state_[2];
    cost += dq.squaredNorm();
    gradient_q[q.size() - 1] += 2 * dq * 1.0 / (dt * dt);
    gradient_q[q.size() - 2] += 2 * dq * (-2.0) / (dt * dt);
    gradient_q[q.size() - 3] += 2 * dq * 1.0 / (dt * dt);
    if (optimize_time_) gt += (fuel_regression::flag('G')?4.0:1.0) * dq.dot(q_1 - 2 * q_2 + q_3) / (-dt * dt * dt);
  }
}

void BsplineOptimizer::calcWaypointsCost(const vector<Eigen::Vector3d>& q, double& cost,
                                         vector<Eigen::Vector3d>& gradient_q) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);

  Eigen::Vector3d q1, q2, q3, dq;

  // for (auto wp : waypoints_) {
  for (int i = 0; i < waypoints_.size(); ++i) {
    Eigen::Vector3d waypt = waypoints_[i];
    int idx = waypt_idx_[i];

    q1 = q[idx];
    q2 = q[idx + 1];
    q3 = q[idx + 2];

    dq = 1 / 6.0 * (q1 + 4 * q2 + q3) - waypt;
    cost += dq.squaredNorm();

    gradient_q[idx] += dq * (2.0 / 6.0);      // 2*dq*(1/6)
    gradient_q[idx + 1] += dq * (8.0 / 6.0);  // 2*dq*(4/6)
    gradient_q[idx + 2] += dq * (2.0 / 6.0);
  }
}

/* use the uniformly sampled points on a geomertic path to guide the
 * trajectory. For each control points to be optimized, it is assigned a
 * guiding point on the path and the distance between them is penalized */
void BsplineOptimizer::calcGuideCost(const vector<Eigen::Vector3d>& q, double& cost,
                                     vector<Eigen::Vector3d>& gradient_q) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);

  int end_idx = q.size() - order_;

  for (int i = order_; i < end_idx; i++) {
    Eigen::Vector3d gpt = guide_pts_[i - order_];
    cost += (q[i] - gpt).squaredNorm();
    gradient_q[i] += 2 * (q[i] - gpt);
  }
}

void BsplineOptimizer::calcViewCost(const vector<Eigen::Vector3d>& q, double& cost,
                                    vector<Eigen::Vector3d>& gradient_q) {
  cost = 0.0;
  Eigen::Vector3d zero(0, 0, 0);
  std::fill(gradient_q.begin(), gradient_q.end(), zero);
  Eigen::Vector3d p = view_cons_.pt_;
  Eigen::Vector3d v = view_cons_.dir_.normalized();
  Eigen::Matrix3d vvT = v * v.transpose();
  Eigen::Matrix3d I_vvT = Eigen::Matrix3d::Identity() - vvT;

  // prependicular cost, increase visibility of points before blocked point
  int i = view_cons_.idx_;
  Eigen::Vector3d dn = (q[i] - p) - ((q[i] - p).dot(v)) * v;
  cost += dn.squaredNorm();
  gradient_q[i] += 2 * I_vvT * dn;
  double norm_dn = dn.norm();

  // parallel cost, increase projection along view direction
  Eigen::Vector3d dl = ((q[i] - p).dot(v)) * v;
  double norm_dl = dl.norm();
  double safe_dist = view_cons_.dir_.norm();
  if (norm_dl < safe_dist) {
    cost += wnl_ * pow(norm_dl - safe_dist, 2);
    gradient_q[i] += wnl_ * 2 * (norm_dl - safe_dist) * vvT * dl / norm_dl;
  }
}

void BsplineOptimizer::calcTimeCost(const double& dt, double& cost, double& gt) {
  // Min time
  double duration = (point_num_ - order_) * dt;
  cost = duration;
  gt = double(point_num_ - order_);

  // Time lower bound
  if (time_lb_ > 0 && duration < time_lb_) {
    static const double w_lb = 10;
    cost += w_lb * pow(duration - time_lb_, 2);
    gt += w_lb * 2 * (duration - time_lb_) * (point_num_ - order_);
  }
}

void BsplineOptimizer::combineCost(const std::vector<double>& x, std::vector<double>& grad,
                                   double& f_combine) {
  {
    /* Convert the NLopt format vector to control points. */

    // This solver can support 1D-3D B-spline optimization, but we use Vector3d to store
    // each control point
    // For 1D case, the second and third elements are zero, and similar for the 2D case.

    // Deprecated
    // for (int i = 0; i < order_; i++)
    // {
    //   for (int j = 0; j < dim_; ++j)
    //     g_q_[i][j] = control_points_(i, j);
    //   for (int j = dim_; j < 3; ++j)
    //     g_q_[i][j] = 0.0;
    // }

    // for (int i = 0; i < variable_num_ / dim_; i++)
    // {
    //   for (int j = 0; j < dim_; ++j)
    //     g_q_[i + order_][j] = x[dim_ * i + j];
    //   for (int j = dim_; j < 3; ++j)
    //     g_q_[i + order_][j] = 0.0;
    // }

    // if (!(cost_function_ & BOUNDARY))
    // {
    //   for (int i = 0; i < order_; i++)
    //   {
    //     for (int j = 0; j < dim_; ++j)
    //       g_q_[order_ + variable_num_ / dim_ + i][j] = control_points_(control_points_.rows() - order_
    //       + i, j);
    //     for (int j = dim_; j < 3; ++j)
    //       g_q_[order_ + variable_num_ / dim_ + i][j] = 0.0;
    //   }
    // }
  }

  ros::Time t1 = ros::Time::now();

  for (int i = 0; i < point_num_; ++i) {
    for (int j = 0; j < dim_; ++j)
      g_q_[i][j] = x[dim_ * i + j];
    for (int j = dim_; j < 3; ++j)
      g_q_[i][j] = 0.0;
  }
  const double dt = optimize_time_ ? x[variable_num_ - 1] : knot_span_;

  f_combine = 0.0;
  grad.resize(variable_num_);
  fill(grad.begin(), grad.end(), 0.0);

  if (cost_function_ & SMOOTHNESS) {
    double f_smoothness = 0.0, gt_smoothness = 0.0;
    calcSmoothnessCost(g_q_, dt, f_smoothness, g_smoothness_, gt_smoothness);
    f_combine += ld_smooth_ * f_smoothness;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += ld_smooth_ * g_smoothness_[i](j);
    if (optimize_time_) grad[variable_num_ - 1] += ld_smooth_ * gt_smoothness;
  }
  if (cost_function_ & DISTANCE) {
    double f_distance = 0.0;
    calcDistanceCost(g_q_, f_distance, g_distance_);
    f_combine += ld_dist_ * f_distance;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += ld_dist_ * g_distance_[i](j);
  }
  if (cost_function_ & FEASIBILITY) {
    double f_feasibility = 0.0, gt_feasibility = 0.0;
    calcFeasibilityCost(g_q_, dt, f_feasibility, g_feasibility_, gt_feasibility);
    f_combine += ld_feasi_ * f_feasibility;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += ld_feasi_ * g_feasibility_[i](j);
    if (optimize_time_) grad[variable_num_ - 1] += ld_feasi_ * gt_feasibility;
  }
  if (cost_function_ & START) {
    double f_start = 0.0, gt_start = 0.0;
    calcStartCost(g_q_, dt, f_start, g_start_, gt_start);
    f_combine += ld_start_ * f_start;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += ld_start_ * g_start_[i](j);
    if (optimize_time_) grad[variable_num_ - 1] += ld_start_ * gt_start;
  }
  if (cost_function_ & END) {
    double f_end = 0.0, gt_end = 0.0;
    calcEndCost(g_q_, dt, f_end, g_end_, gt_end);
    f_combine += ld_end_ * f_end;
    for (int i = point_num_ - 3; i < point_num_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += ld_end_ * g_end_[i](j);
    if (optimize_time_) grad[variable_num_ - 1] += ld_end_ * gt_end;
  }
  if (cost_function_ & GUIDE) {
    double f_guide = 0.0;
    calcGuideCost(g_q_, f_guide, g_guide_);
    f_combine += ld_guide_ * f_guide;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += ld_guide_ * g_guide_[i](j);
  }
  if (cost_function_ & WAYPOINTS) {
    double f_waypoints = 0.0;
    calcWaypointsCost(g_q_, f_waypoints, g_waypoints_);
    f_combine += ld_waypt_ * f_waypoints;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += ld_waypt_ * g_waypoints_[i](j);
  }
  if (cost_function_ & VIEWCONS) {
    double f_view = 0.0;
    calcViewCost(g_q_, f_view, g_view_);
    f_combine += ld_view_ * f_view;
    for (int i = 0; i < point_num_; i++)
      for (int j = 0; j < dim_; j++)
        grad[dim_ * i + j] += ld_view_ * g_view_[i](j);
  }
  if (cost_function_ & MINTIME) {
    double f_time = 0.0, gt_time = 0.0;
    calcTimeCost(dt, f_time, gt_time);
    f_combine += ld_time_ * f_time;
    grad[variable_num_ - 1] += ld_time_ * gt_time;
  }

  comb_time += (ros::Time::now() - t1).toSec();

  // // Join thread and retrive cost/gradient
  // for (int i = 0; i < cost_thread.size(); ++i)
  //   cost_thread[i].join();

  // if (cost_function_ & SMOOTHNESS)
  // {
  //   f_combine += ld_smooth_ * f_smoothness;
  //   for (int i = 0; i < point_num_; i++)
  //     for (int j = 0; j < dim_; j++)
  //       grad[dim_ * i + j] += ld_smooth_ * g_smoothness_[i](j);
  //   if (optimize_time_)
  //     grad[variable_num_ - 1] += ld_smooth_ * gt_smoothness;
  // }
  // if (cost_function_ & DISTANCE)
  // {
  //   f_combine += ld_dist_ * f_distance;
  //   for (int i = 0; i < point_num_; i++)
  //     for (int j = 0; j < dim_; j++)
  //       grad[dim_ * i + j] += ld_dist_ * g_distance_[i](j);
  // }
  // if (cost_function_ & FEASIBILITY)
  // {
  //   f_combine += ld_feasi_ * f_feasibility;
  //   for (int i = 0; i < point_num_; i++)
  //     for (int j = 0; j < dim_; j++)
  //       grad[dim_ * i + j] += ld_feasi_ * g_feasibility_[i](j);
  //   if (optimize_time_)
  //     grad[variable_num_ - 1] += ld_feasi_ * gt_feasibility;
  // }

  /*  print cost  */
  // if ((cost_function_ & WAYPOINTS) && iter_num_ % 10 == 0) {
  //   cout << iter_num_ << ", total: " << f_combine << ", acc: " << ld_view_ * f_view
  //        << ", waypt: " << ld_waypt_ * f_waypoints << endl;
  // }

  // if (optimization_phase_ == SECOND_PHASE) {
  //  << ", smooth: " << ld_smooth_ * f_smoothness
  //  << " , dist:" << ld_dist_ * f_distance
  //  << ", fea: " << ld_feasi_ * f_feasibility << endl;
  // << ", end: " << ld_bound_ * f_endpoint
  // << ", guide: " << ld_guide_ * f_guide
  // }
}

double BsplineOptimizer::costFunction(const std::vector<double>& x, std::vector<double>& grad,
                                      void* func_data) {
  BsplineOptimizer* opt = reinterpret_cast<BsplineOptimizer*>(func_data);
  double cost;
  opt->combineCost(x, grad, cost);
  opt->iter_num_++;

  /* save the min cost result */
  if (cost < opt->min_cost_) {
    opt->min_cost_ = cost;
    opt->best_variable_ = x;
    // std::cout << cost << ", ";
  }
  return cost;

  // /* evaluation */
  // ros::Time te1 = ros::Time::now();
  // double time_now = (te1 - opt->time_start_).toSec();
  // opt->vec_time_.push_back(time_now);
  // if (opt->vec_cost_.size() == 0)
  // {
  //   opt->vec_cost_.push_back(f_combine);
  // }
  // else if (opt->vec_cost_.back() > f_combine)
  // {
  //   opt->vec_cost_.push_back(f_combine);
  // }
  // else
  // {
  //   opt->vec_cost_.push_back(opt->vec_cost_.back());
  // }
}

vector<Eigen::Vector3d> BsplineOptimizer::matrixToVectors(const Eigen::MatrixXd& ctrl_pts) {
  vector<Eigen::Vector3d> ctrl_q;
  for (int i = 0; i < ctrl_pts.rows(); ++i) {
    ctrl_q.push_back(ctrl_pts.row(i));
  }
  return ctrl_q;
}

Eigen::MatrixXd BsplineOptimizer::getControlPoints() {
  return this->control_points_;
}

bool BsplineOptimizer::isQuadratic() {
  if (cost_function_ == GUIDE_PHASE) {
    return true;
  } else if (cost_function_ == SMOOTHNESS) {
    return true;
  } else if (cost_function_ == (SMOOTHNESS | WAYPOINTS)) {
    return true;
  }
  return false;
}

}  // namespace fast_planner