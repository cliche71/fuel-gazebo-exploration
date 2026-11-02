#include <plan_env/regression_evidence.h>

#include <plan_manage/planner_manager.h>
#include <exploration_manager/fast_exploration_manager.h>
#include <traj_utils/planning_visualization.h>

#include <exploration_manager/fast_exploration_fsm.h>
#include <exploration_manager/expl_data.h>
#include <plan_env/edt_environment.h>
#include <plan_env/sdf_map.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <sstream>

using Eigen::Vector4d;

namespace fast_planner {

namespace {
void officialPublishState(const std::string& state) {
  static ros::NodeHandle nh("~");
  static ros::Publisher pub = nh.advertise<std_msgs::String>("state", 1, true);
  std_msgs::String message; message.data = state; pub.publish(message);
}

// Observation only: reproduce the original Astar first expansion. In
// particular UNKNOWN in the start cell itself is not a new veto condition.
void officialPublishStartRegion(SDFMap& map, const Eigen::Vector3d& start,
    double step, size_t frontiers) {
  int clear = 0, unknown = 0, inflated = 0, outside = 0;
  std::ostringstream edges;
  int edge_number = 0;
  for (int x=-1; x<=1; ++x) for (int y=-1; y<=1; ++y) for (int z=-1; z<=1; ++z) {
    if (!x && !y && !z) continue;
    const Eigen::Vector3d delta = step*Eigen::Vector3d(x,y,z);
    const Eigen::Vector3d end = start+delta;
    Eigen::Vector3d block = end;
    std::string reason = "clear";
    if (!map.isInMap(end) || !map.isInBox(end)) reason = "outside";
    else if (map.getInflateOccupancy(end) == 1) reason = "inflated";
    else if (map.getOccupancy(end) == SDFMap::UNKNOWN) reason = "unknown";
    else {
      const Eigen::Vector3d direction = delta.normalized();
      for (double distance=0.1; distance<delta.norm(); distance+=0.1) {
        block = start + distance*direction;
        if (map.getInflateOccupancy(block) == 1) { reason="inflated"; break; }
        if (map.getOccupancy(block) == SDFMap::UNKNOWN) { reason="unknown"; break; }
      }
    }
    if (reason == "clear") ++clear;
    else if (reason == "unknown") ++unknown;
    else if (reason == "inflated") ++inflated;
    else ++outside;
    if (edge_number++) edges << ',';
    edges << "{\"direction\":["<<x<<','<<y<<','<<z<<"],\"result\":\""<<reason
          <<"\",\"first_block\":["<<block.x()<<','<<block.y()<<','<<block.z()<<"]}";
  }
  const bool ready = clear > 0 && frontiers > 0;
  std::ostringstream report;
  report.precision(12);
  report << "{\"clear_edges\":"<<clear<<",\"unknown_edges\":"<<unknown
         <<",\"inflated_edges\":"<<inflated<<",\"outside_edges\":"<<outside
         <<",\"frontier_groups\":"<<frontiers<<",\"start\":["<<start.x()<<','
         <<start.y()<<','<<start.z()<<"],\"start_occupancy\":"
         <<(map.isInMap(start)?map.getOccupancy(start):-1)<<",\"ready\":"
         <<(ready?"true":"false")<<",\"edges\":["<<edges.str()<<"]}";
  static ros::NodeHandle nh("~");
  static ros::Publisher ready_pub=nh.advertise<std_msgs::Bool>("start_region_ready",1,true);
  static ros::Publisher report_pub=nh.advertise<std_msgs::String>("start_region_report",1,true);
  std_msgs::Bool ready_message; ready_message.data=ready; ready_pub.publish(ready_message);
  std_msgs::String report_message; report_message.data=report.str(); report_pub.publish(report_message);
}
}  // namespace

void FastExplorationFSM::init(ros::NodeHandle& nh) {
  fp_.reset(new FSMParam);
  fd_.reset(new FSMData);

  /*  Fsm param  */
  nh.param("fsm/thresh_replan1", fp_->replan_thresh1_, -1.0);
  nh.param("fsm/thresh_replan2", fp_->replan_thresh2_, -1.0);
  nh.param("fsm/thresh_replan3", fp_->replan_thresh3_, -1.0);
  nh.param("fsm/replan_time", fp_->replan_time_, -1.0);

  /* Initialize main modules */
  expl_manager_.reset(new FastExplorationManager);
  expl_manager_->initialize(nh);
  visualization_.reset(new PlanningVisualization(nh));

  planner_manager_ = expl_manager_->planner_manager_;
  state_ = EXPL_STATE::INIT;
  fd_->have_odom_ = false;
  fd_->state_str_ = { "INIT", "WAIT_TRIGGER", "PLAN_TRAJ", "PUB_TRAJ", "EXEC_TRAJ", "FINISH" };
  fd_->static_state_ = true;
  fd_->trigger_ = false;

  /* Ros sub, pub and timer */
  exec_timer_ = nh.createTimer(ros::Duration(0.01), &FastExplorationFSM::FSMCallback, this);
  safety_timer_ = nh.createTimer(ros::Duration(0.05), &FastExplorationFSM::safetyCallback, this);
  frontier_timer_ = nh.createTimer(ros::Duration(0.5), &FastExplorationFSM::frontierCallback, this);

  trigger_sub_ =
      nh.subscribe("/waypoint_generator/waypoints", 1, &FastExplorationFSM::triggerCallback, this);
  odom_sub_ = nh.subscribe("/odom_world", 1, &FastExplorationFSM::odometryCallback, this);

  nh.param("fsm/safety_stop_before_hit",safety_stop_enabled_,false);
  safety_stop_pub_=nh.advertise<std_msgs::Header>("/planning/safety_stop",10);
  replan_pub_ = nh.advertise<std_msgs::Empty>("/planning/replan", 10);
  new_pub_ = nh.advertise<std_msgs::Empty>("/planning/new", 10);
  bspline_pub_ = nh.advertise<bspline::Bspline>("/planning/bspline", 10);
}

void FastExplorationFSM::FSMCallback(const ros::TimerEvent& e) {
  officialPublishState(fd_->state_str_[int(state_)]);
  ROS_INFO_STREAM_THROTTLE(1.0, "[FSM]: state: " << fd_->state_str_[int(state_)]);

  switch (state_) {
    case INIT: {
      // Wait for odometry ready
      if (!fd_->have_odom_) {
        ROS_WARN_THROTTLE(1.0, "no odom.");
        return;
      }
      // Go to wait trigger when odom is ok
      transitState(WAIT_TRIGGER, "FSM");
      break;
    }

    case WAIT_TRIGGER: {
      // Do nothing but wait for trigger
      ROS_WARN_THROTTLE(1.0, "wait for trigger.");
      break;
    }

    case FINISH: {
      ROS_INFO_THROTTLE(1.0, "finish exploration.");
      break;
    }

    case PLAN_TRAJ: {
      if (fd_->static_state_) {
        // Plan from static state (hover)
        fd_->start_pt_ = fd_->odom_pos_;
        fd_->start_vel_ = fd_->odom_vel_;
        fd_->start_acc_.setZero();

        fd_->start_yaw_(0) = fd_->odom_yaw_;
        fd_->start_yaw_(1) = fd_->start_yaw_(2) = 0.0;
      } else {
        // Replan from non-static state, starting from 'replan_time' seconds later
        LocalTrajData* info = &planner_manager_->local_data_;
        double t_r = (ros::Time::now() - info->start_time_).toSec() + fp_->replan_time_;

        bool stopping_hold=false;
        if(safety_stop_enabled_&&safety_stop_id_==info->traj_id_){
          t_r=std::min(t_r,safety_stop_phase_);stopping_hold=t_r>=safety_stop_phase_-1e-8;
        }
        fd_->start_pt_ = info->position_traj_.evaluateDeBoorT(t_r);
        fd_->start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_r);
        fd_->start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_r);
        fd_->start_yaw_(0) = info->yaw_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(1) = info->yawdot_traj_.evaluateDeBoorT(t_r)[0];
        fd_->start_yaw_(2) = info->yawdotdot_traj_.evaluateDeBoorT(t_r)[0];
        if(stopping_hold){fd_->start_vel_.setZero();fd_->start_acc_.setZero();
          fd_->start_yaw_(1)=fd_->start_yaw_(2)=0.;}
      }

      // Inform traj_server the replanning
      if(fuel_regression::active()){fuel_regression::J d;auto& l=planner_manager_->local_data_;d.put("old_curve_id",l.traj_id_);d.put("old_phase",(ros::Time::now()-l.start_time_).toSec());d.put("old_duration",l.duration_);d.add_child("p",fuel_regression::vector(fd_->start_pt_));d.add_child("v",fuel_regression::vector(fd_->start_vel_));d.add_child("a",fuel_regression::vector(fd_->start_acc_));d.add_child("yaw_derivatives",fuel_regression::vector(fd_->start_yaw_));fuel_regression::emit("planning_replan_callback",d);}
      replan_pub_.publish(std_msgs::Empty());
      if(planner_manager_->executionGuardEnabled())
        pre_publication_data_.reset(new LocalTrajData(planner_manager_->local_data_));
      int res = callExplorationPlanner();
      if(res!=SUCCEED)pre_publication_data_.reset();
      if (res == SUCCEED) {
        transitState(PUB_TRAJ, "FSM");
      } else if (res == NO_FRONTIER) {
        transitState(FINISH, "FSM");
        fd_->static_state_ = true;
        clearVisMarker();
      } else if (res == FAIL) {
        // Still in PLAN_TRAJ state, keep replanning
        ROS_WARN("plan fail");
        fd_->static_state_ = true;
      }
      break;
    }

    case PUB_TRAJ: {
      double dt = (ros::Time::now() - fd_->newest_traj_.start_time).toSec();
      if (dt > 0) {
      planner_manager_->traceClearance("before_publication",std::max(0.0,dt));
      if(planner_manager_->executionGuardEnabled()&&!planner_manager_->executionPrefix(dt).safe){
        fuel_regression::J d;d.put("rejected_id",planner_manager_->local_data_.traj_id_);
        if(pre_publication_data_)planner_manager_->local_data_=*pre_publication_data_;
        auto& restored=planner_manager_->local_data_;
        d.put("restored_id",restored.traj_id_);d.put("restored_duration",restored.duration_);
        d.put("restored_start_time",restored.start_time_.toSec());
        d.add_child("restored_start_position",fuel_regression::vector(restored.start_pos_));
        fuel_regression::J splines;
        for(auto* spline:{&restored.position_traj_,&restored.velocity_traj_,&restored.acceleration_traj_,
            &restored.yaw_traj_,&restored.yawdot_traj_,&restored.yawdotdot_traj_}){
          fuel_regression::J one;one.add_child("points",fuel_regression::eigen(spline->getControlPoint()));
          one.add_child("knots",fuel_regression::vector(spline->getKnot()));splines.push_back({"",one});
        }
        d.add_child("restored_six_splines",splines);fuel_regression::emit("publication_rollback",d);
        pre_publication_data_.reset();
        if(safety_stop_enabled_&&planner_manager_->local_data_.traj_id_>0&&
            planner_manager_->local_data_.duration_>0.){
          double distance;if(!planner_manager_->checkTrajCollision(distance))sendSafetyStop();
        }
        fd_->static_state_=true;transitState(PLAN_TRAJ,"unsafePublication");return;
      }
        if(fuel_regression::active()){fuel_regression::J d;d.put("trajectory_id",fd_->newest_traj_.traj_id);d.put("start_time",fd_->newest_traj_.start_time.toSec());fuel_regression::emit("candidate_published",d);}
      bspline_pub_.publish(fd_->newest_traj_);
      pre_publication_data_.reset();safety_stop_id_=-1;
        fd_->static_state_ = false;
        transitState(EXEC_TRAJ, "FSM");

        thread vis_thread(&FastExplorationFSM::visualize, this);
        vis_thread.detach();
      }
      break;
    }

    case EXEC_TRAJ: {
      LocalTrajData* info = &planner_manager_->local_data_;
      double t_cur = (ros::Time::now() - info->start_time_).toSec();

      // Replan if traj is almost fully executed
      double time_to_end = info->duration_ - t_cur;
      if (time_to_end < fp_->replan_thresh1_) {
        transitState(PLAN_TRAJ, "FSM");
        ROS_WARN("Replan: traj fully executed=================================");
        return;
      }
      // Replan if next frontier to be visited is covered
      if (t_cur > fp_->replan_thresh2_ && expl_manager_->frontier_finder_->isFrontierCovered()) {
        transitState(PLAN_TRAJ, "FSM");
        ROS_WARN("Replan: cluster covered=====================================");
        return;
      }
      // Replan after some time
      if (t_cur > fp_->replan_thresh3_ && !classic_) {
        transitState(PLAN_TRAJ, "FSM");
        ROS_WARN("Replan: periodic call=======================================");
      }
      break;
    }
  }
}

int FastExplorationFSM::callExplorationPlanner() {
  ros::Time time_r = ros::Time::now() + ros::Duration(fp_->replan_time_);

  int res = expl_manager_->planExploreMotion(fd_->start_pt_, fd_->start_vel_, fd_->start_acc_,
                                             fd_->start_yaw_);
  classic_ = false;

  // int res = expl_manager_->classicFrontier(fd_->start_pt_, fd_->start_yaw_[0]);
  // classic_ = true;

  // int res = expl_manager_->rapidFrontier(fd_->start_pt_, fd_->start_vel_, fd_->start_yaw_[0],
  // classic_);

  if (res == SUCCEED) {
    auto info = &planner_manager_->local_data_;
    info->start_time_ = (ros::Time::now() - time_r).toSec() > 0 ? ros::Time::now() : time_r;

    bspline::Bspline bspline;
    bspline.order = planner_manager_->pp_.bspline_degree_;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;
    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    for (int i = 0; i < pos_pts.rows(); ++i) {
      geometry_msgs::Point pt;
      pt.x = pos_pts(i, 0);
      pt.y = pos_pts(i, 1);
      pt.z = pos_pts(i, 2);
      bspline.pos_pts.push_back(pt);
    }
    Eigen::VectorXd knots = info->position_traj_.getKnot();
    for (int i = 0; i < knots.rows(); ++i) {
      bspline.knots.push_back(knots(i));
    }
    Eigen::MatrixXd yaw_pts = info->yaw_traj_.getControlPoint();
    for (int i = 0; i < yaw_pts.rows(); ++i) {
      double yaw = yaw_pts(i, 0);
      bspline.yaw_pts.push_back(yaw);
    }
    bspline.yaw_dt = info->yaw_traj_.getKnotSpan();
    fd_->newest_traj_ = bspline;
  }
  return res;
}

void FastExplorationFSM::visualize() {
  auto info = &planner_manager_->local_data_;
  auto plan_data = &planner_manager_->plan_data_;
  auto ed_ptr = expl_manager_->ed_;

  // Draw updated box
  // Vector3d bmin, bmax;
  // planner_manager_->edt_environment_->sdf_map_->getUpdatedBox(bmin, bmax);
  // visualization_->drawBox((bmin + bmax) / 2.0, bmax - bmin, Vector4d(0, 1, 0, 0.3), "updated_box", 0,
  // 4);

  // Draw frontier
  static int last_ftr_num = 0;
  for (int i = 0; i < ed_ptr->frontiers_.size(); ++i) {
    visualization_->drawCubes(ed_ptr->frontiers_[i], 0.1,
                              visualization_->getColor(double(i) / ed_ptr->frontiers_.size(), 0.4),
                              "frontier", i, 4);
    // visualization_->drawBox(ed_ptr->frontier_boxes_[i].first, ed_ptr->frontier_boxes_[i].second,
    //                         Vector4d(0.5, 0, 1, 0.3), "frontier_boxes", i, 4);
  }
  for (int i = ed_ptr->frontiers_.size(); i < last_ftr_num; ++i) {
    visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
    // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
    // "frontier_boxes", i, 4);
  }
  last_ftr_num = ed_ptr->frontiers_.size();
  // for (int i = 0; i < ed_ptr->dead_frontiers_.size(); ++i)
  //   visualization_->drawCubes(ed_ptr->dead_frontiers_[i], 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier",
  //                             i, 4);
  // for (int i = ed_ptr->dead_frontiers_.size(); i < 5; ++i)
  //   visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 0.5), "dead_frontier", i, 4);

  // Draw global top viewpoints info
  // visualization_->drawSpheres(ed_ptr->points_, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
  // visualization_->drawLines(ed_ptr->global_tour_, 0.07, Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);
  // visualization_->drawLines(ed_ptr->points_, ed_ptr->views_, 0.05, Vector4d(0, 1, 0.5, 1), "view", 0, 6);
  // visualization_->drawLines(ed_ptr->points_, ed_ptr->averages_, 0.03, Vector4d(1, 0, 0, 1),
  // "point-average", 0, 6);

  // Draw local refined viewpoints info
  // visualization_->drawSpheres(ed_ptr->refined_points_, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->refined_views_, 0.05,
  //                           Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_tour_, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_views1_, ed_ptr->refined_views2_, 0.04, Vector4d(0, 0, 0,
  // 1),
  //                           "refined_view", 0, 6);
  // visualization_->drawLines(ed_ptr->refined_points_, ed_ptr->unrefined_points_, 0.05, Vector4d(1, 1,
  // 0, 1),
  //                           "refine_pair", 0, 6);
  // for (int i = 0; i < ed_ptr->n_points_.size(); ++i)
  //   visualization_->drawSpheres(ed_ptr->n_points_[i], 0.1,
  //                               visualization_->getColor(double(ed_ptr->refined_ids_[i]) /
  //                               ed_ptr->frontiers_.size()),
  //                               "n_points", i, 6);
  // for (int i = ed_ptr->n_points_.size(); i < 15; ++i)
  //   visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 0, 1), "n_points", i, 6);

  // Draw trajectory
  // visualization_->drawSpheres({ ed_ptr->next_goal_ }, 0.3, Vector4d(0, 1, 1, 1), "next_goal", 0, 6);
  visualization_->drawBspline(info->position_traj_, 0.1, Vector4d(1.0, 0.0, 0.0, 1), false, 0.15,
                              Vector4d(1, 1, 0, 1));
  // visualization_->drawSpheres(plan_data->kino_path_, 0.1, Vector4d(1, 0, 1, 1), "kino_path", 0, 0);
  // visualization_->drawLines(ed_ptr->path_next_goal_, 0.05, Vector4d(0, 1, 1, 1), "next_goal", 1, 6);
}

void FastExplorationFSM::clearVisMarker() {
  // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0.5, 0, 1), "points", 0, 6);
  // visualization_->drawLines({}, 0.07, Vector4d(0, 0.5, 0, 1), "global_tour", 0, 6);
  // visualization_->drawSpheres({}, 0.2, Vector4d(0, 0, 1, 1), "refined_pts", 0, 6);
  // visualization_->drawLines({}, {}, 0.05, Vector4d(0.5, 0, 1, 1), "refined_view", 0, 6);
  // visualization_->drawLines({}, 0.07, Vector4d(0, 0, 1, 1), "refined_tour", 0, 6);
  // visualization_->drawSpheres({}, 0.1, Vector4d(0, 0, 1, 1), "B-Spline", 0, 0);

  // visualization_->drawLines({}, {}, 0.03, Vector4d(1, 0, 0, 1), "current_pose", 0, 6);
}

void FastExplorationFSM::frontierCallback(const ros::TimerEvent& e) {
  static int delay = 0;
  if (++delay < 5) return;

  if (state_ == WAIT_TRIGGER || state_ == FINISH) {
    auto ft = expl_manager_->frontier_finder_;
    auto ed = expl_manager_->ed_;
    ft->searchFrontiers();
    ft->computeFrontiersToVisit();
    ft->updateFrontierCostMatrix();

    ft->getFrontiers(ed->frontiers_);
    ft->getFrontierBoxes(ed->frontier_boxes_);

    double first_step;
    ros::NodeHandle("~").param("astar/resolution_astar", first_step, 0.2);
    officialPublishStartRegion(*planner_manager_->edt_environment_->sdf_map_,
        fd_->odom_pos_, first_step, ed->frontiers_.size());

    // Draw frontier and bounding box
    for (int i = 0; i < ed->frontiers_.size(); ++i) {
      visualization_->drawCubes(ed->frontiers_[i], 0.1,
                                visualization_->getColor(double(i) / ed->frontiers_.size(), 0.4),
                                "frontier", i, 4);
      // visualization_->drawBox(ed->frontier_boxes_[i].first, ed->frontier_boxes_[i].second,
      // Vector4d(0.5, 0, 1, 0.3),
      //                         "frontier_boxes", i, 4);
    }
    for (int i = ed->frontiers_.size(); i < 50; ++i) {
      visualization_->drawCubes({}, 0.1, Vector4d(0, 0, 0, 1), "frontier", i, 4);
      // visualization_->drawBox(Vector3d(0, 0, 0), Vector3d(0, 0, 0), Vector4d(1, 0, 0, 0.3),
      // "frontier_boxes", i, 4);
    }
  }

  // if (!fd_->static_state_)
  // {
  //   static double astar_time = 0.0;
  //   static int astar_num = 0;
  //   auto t1 = ros::Time::now();

  //   planner_manager_->path_finder_->reset();
  //   planner_manager_->path_finder_->setResolution(0.4);
  //   if (planner_manager_->path_finder_->search(fd_->odom_pos_, Vector3d(-5, 0, 1)))
  //   {
  //     auto path = planner_manager_->path_finder_->getPath();
  //     visualization_->drawLines(path, 0.05, Vector4d(1, 0, 0, 1), "astar", 0, 6);
  //     auto visit = planner_manager_->path_finder_->getVisited();
  //     visualization_->drawCubes(visit, 0.3, Vector4d(0, 0, 1, 0.4), "astar-visit", 0, 6);
  //   }
  //   astar_num += 1;
  //   astar_time = (ros::Time::now() - t1).toSec();
  //   ROS_WARN("Average astar time: %lf", astar_time);
  // }
}

void FastExplorationFSM::triggerCallback(const nav_msgs::PathConstPtr& msg) {
  if (msg->poses.empty()) return;  // Reject an invalid transport message.
  if (msg->poses[0].pose.position.z < -0.1) return;
  if (state_ != WAIT_TRIGGER) return;
  fd_->trigger_ = true;
  cout << "Triggered!" << endl;
  transitState(PLAN_TRAJ, "triggerCallback");
}

void FastExplorationFSM::sendSafetyStop(){
  auto& l=planner_manager_->local_data_;
  double requested=planner_manager_->safetyStopPhase();
  if(safety_stop_id_==l.traj_id_)requested=std::min(requested,safety_stop_phase_);
  safety_stop_id_=l.traj_id_;safety_stop_phase_=requested;
  std_msgs::Header msg;msg.seq=l.traj_id_;msg.stamp=l.start_time_+ros::Duration(safety_stop_phase_);
  msg.frame_id="execution_prefix_clearance";safety_stop_pub_.publish(msg);
  fuel_regression::J d;d.put("trajectory_id",l.traj_id_);d.put("stop_phase",safety_stop_phase_);
  d.put("deadline",msg.stamp.toSec());d.put("curve_start",l.start_time_.toSec());
  fuel_regression::emit("safety_stop_requested",d);
}

void FastExplorationFSM::safetyCallback(const ros::TimerEvent& e) {
  if (state_ == EXPL_STATE::EXEC_TRAJ) {
    // Check safety and trigger replan if necessary
    double dist;
    bool safe = planner_manager_->checkTrajCollision(dist);
    if (!safe) {
      ROS_WARN("Replan: collision detected==================================");
      if(fuel_regression::active()){fuel_regression::J d;d.put("cause","collision_prediction");d.put("curve_id",planner_manager_->local_data_.traj_id_);d.put("phase",(ros::Time::now()-planner_manager_->local_data_.start_time_).toSec());fuel_regression::emit("safety_callback",d);}
      if(safety_stop_enabled_)sendSafetyStop();
      transitState(PLAN_TRAJ, "safetyCallback");
    }
  }
}

void FastExplorationFSM::odometryCallback(const nav_msgs::OdometryConstPtr& msg) {
  fd_->odom_pos_(0) = msg->pose.pose.position.x;
  fd_->odom_pos_(1) = msg->pose.pose.position.y;
  fd_->odom_pos_(2) = msg->pose.pose.position.z;

  fd_->odom_vel_(0) = msg->twist.twist.linear.x;
  fd_->odom_vel_(1) = msg->twist.twist.linear.y;
  fd_->odom_vel_(2) = msg->twist.twist.linear.z;

  fd_->odom_orient_.w() = msg->pose.pose.orientation.w;
  fd_->odom_orient_.x() = msg->pose.pose.orientation.x;
  fd_->odom_orient_.y() = msg->pose.pose.orientation.y;
  fd_->odom_orient_.z() = msg->pose.pose.orientation.z;

  Eigen::Vector3d rot_x = fd_->odom_orient_.toRotationMatrix().block<3, 1>(0, 0);
  fd_->odom_yaw_ = atan2(rot_x(1), rot_x(0));

  fd_->have_odom_ = true;
}

void FastExplorationFSM::transitState(EXPL_STATE new_state, string pos_call) {
  int pre_s = int(state_);
  state_ = new_state;
  officialPublishState(fd_->state_str_[int(state_)]);
  cout << "[" + pos_call + "]: from " + fd_->state_str_[pre_s] + " to " + fd_->state_str_[int(new_state)]
       << endl;
}
}  // namespace fast_planner
