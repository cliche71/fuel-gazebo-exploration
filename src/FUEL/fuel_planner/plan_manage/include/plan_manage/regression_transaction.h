#pragma once
#include <plan_manage/plan_container.hpp>
namespace fast_planner {
struct CandidateTrajectory {
    LocalTrajData& live;
    LocalTrajData saved;
    bool committed;
    explicit CandidateTrajectory(LocalTrajData& data)
        : live(data), saved(data), committed(false) {}
    ~CandidateTrajectory() { if (!committed) live = saved; }
  };
}
