# Differences from official FUEL 662dd23

Version 0.1.3 adds `yaw_motion_limits.json`. Exploration passes its existing
`yd` and `ydd` values to an opt-in yaw check/refinement. The original unconstrained
yaw result passes through if it already meets continuous rate/acceleration and
initial-state checks. Otherwise NLopt SLSQP reuses the same official yaw cost,
with fixed first three yaw control points and whole-span rate/acceleration
inequalities. Velocity extrema include interior roots of the linear acceleration;
acceleration extrema are span endpoints. The returned candidate must independently
pass the same analytic checks. An infeasible initial state or an unverified iterate is rejected. A feasible
braking continuation seeds refinement when possible, and only independently
verified feasible objective iterates are retained if the optimizer ends early.
Commands are never clipped at the interface. Position control points,
knots and duration do not change. Endpoint heading remains the original soft
objective and its residual is logged. This is a documented constraint change,
not a claim of unchanged yaw behavior for previously unsafe trajectories.

The complete position/yaw candidate is committed only on success; failures restore
the previous LocalTrajData. Default spline scalar fields and LocalTrajData fields
are initialized before this value backup. Yaw time lower bounds use periodic
angle differences. The command adapter's existing 3 rad/s rejection threshold
is unchanged; rejected command details now include yaw rate. Default four-argument
planYawExplore callers remain compatible and do not opt into the new constraints.

Version 0.1.2 changes only the Gazebo command adapter, read-only runtime report,
and package version. An isolated stale/out-of-order packet is dropped without
refreshing accepted command health. A bounded 128-sample future queue waits
for the local ROS clock (up to 0.3 s lead at receipt); promotion retains the
message stamp and original receipt wall time. Only already-due samples can
become commands. Accepted commands retain the original 0.3 s simulation-time
validity and 1 s wall-clock watchdog; no waiting/resumption state is added.
The control thread samples time under the command lock. A first-stop diagnostic
and timing counters retain exact age, stamp, clock and trajectory evidence.
No official FUEL source patch, camera, map, motion limit or yaw behavior changes.
This addresses isolated packet timing as a cause of immediate latching; actual
gaps exceeding the existing watchdogs still stop and require a fresh run.

Version 0.1.1: `patches/build_order.json` additionally fixes the legacy
multi_map_visualization dependency by using the package and Catkin exported
message-generation targets. This is a build-order correction only. It adds one
more CMake file to the changes below; no sensor or exploration behavior changes.

The independent workspace starts from the exact committed tree
`662dd23c7b52b258d3c4a0155ff6632118e8984f`, not the existing modified working
tree. The official scene coordinates, frontier grouping, global TSP objective,
local viewpoint selection, A* unknown-space policy, rolling replanning FSM,
position optimization objective, and yaw optimization objective are retained.
This is a documented Gazebo adaptation, not a claim of byte-for-byte identity.

`minimal_compatibility.json` changes eight upstream files:

| Change | Reason and behavioral scope |
| --- | --- |
| Discover NLopt headers and library through CMake | The official build hard-codes `/usr/local`. This only affects dependency discovery. |
| Return success/failure from waypoint trajectory construction | Invalid paths previously continued or could reuse a previous result. The two callers now return the existing planning `FAIL` result. No FSM branch is added. |
| Remove repeated adjacent points and add a midpoint to a two-point path | The official polynomial routine assumes two or more segments; a single segment indexes beyond its selection matrix. Valid multi-point paths retain the same polynomial routine and optimizer. |
| Sample polynomial endpoints with integer indices | Avoid floating-step endpoint overshoot. The same number of spline segments and the same position optimizer are used. |
| Check finite position/yaw inputs and outputs | Rejected results use the existing planning failure path. No map cells are cleared or treated as known. |
| Handle an empty previous yaw waypoint | A constant segment can produce its first zero-displacement waypoint before a previous waypoint exists. The current yaw is retained in this case. Normal forward-looking yaw generation is unchanged. |
| Exclude a zero time interval from NLopt bounds | Spline derivatives divide by the interval. The lower bound is the same compatibility correction tested in the preceding Gazebo work. |
| Initialize NLopt's existing best-result vector | If NLopt throws before its first objective evaluation, use the current finite initial variables rather than an empty or stale vector. The optimization objectives and algorithms are unchanged. |
| Correct acceleration-error time derivatives | For the existing squared boundary acceleration error, the derivative with respect to the knot interval is missing a factor of four at both ends. Only this derivative coefficient is corrected; the objective, weights, solver algorithms, and soft boundary treatment are unchanged. |
| Check the current depth pixel | The official pointer advances before the zero check, so it examines the next pixel or reads beyond the row. Use the depth value already read from the current pixel. The depth fusion policy is unchanged. |
| Initialize map-local index bounds | ROS timers can inspect the map before the first depth update. Initialize valid full-map index bounds as in the official reset routine. All occupancy values remain unknown. |
| Add `traj_server/external_initialization`, default `false` | With the default value, the official two loops still run: 100 commands increasing height by 0.01 m, then 100 decreasing by 0.01 m, at 0.01 s intervals. Explicit `true` delegates this sequence to a connected Gazebo adapter. The trajectory server's subsequent behavior is unchanged. |

No position time stretching, stop-after-each-leg mode, fallback paths, or
unknown-space bypass are included. The optional 0.1.3 yaw refinement adds the
fixed initial yaw state and derivative constraints described above. Additional
`observability.json` changes, when present, only publish diagnostics; their
specific differences are recorded in that patch file.

## Workspace verification

`vendor/upstream_files.json` records each included committed file's SHA-256, Git blob
identity, and mode, plus the fixed Git tree identity. Preparation verifies the
complete included official tree before applying any patch. `archive_paths`
retains both complete source directories, all simulator models and maps, the
license, README, and gitignore. Only the `files/` demonstration images are omitted.
All replacement contexts are
checked in memory before any patched source is written. Original changed files
are saved in the new workspace's `source_backups` directory.

Preparation refuses the existing `~/fuel_ws` and `~/rotors_ws`, their
subdirectories, nonempty unrecognized directories, unexpected source versions,
nonunique patch contexts, and edited prepared sources. A repeated successful run
only verifies the prepared source and patch identities. It does not run Catkin,
start ROS, or change the old flight setup.

## Verification scope

Offline checks exercised preparation from fixed Git objects despite a
deliberately contaminated worktree, repeated verification, edited-source
rejection, nonempty-directory rejection, and protection of the old workspace
paths. The selected position/yaw construction functions were compiled against
the actual official `PolynomialTraj` and `NonUniformBspline` implementations.
Tests covered two- and three-point paths, moving initial velocity, repeated
points, empty/degenerate/nonfinite inputs, and a constant-position yaw segment.
The optimizer was mocked for these construction checks. Further actual NLopt
checks are documented in the delivered validation report. ROS, Gazebo dynamics,
and autonomous exploration were not executed by the offline checks.

For the acceleration derivative correction, write `B = q0 - 2*q1 + q2`,
`r = B/dt^2 - acceleration`, and `cost = r.dot(r)`. The correct time derivative
is `-4 * r.dot(B) / dt^3`; the official expression used `-r.dot(B) / dt^3`.
Central differences at `dt=0.5`, with a `1e-6` difference step, measured:

| Boundary | Official analytic value | Central difference | Corrected analytic value | Corrected absolute difference |
| --- | ---: | ---: | ---: | ---: |
| Start | -1.7852 | -2.7668 | -2.7668 | 1.93e-12 |
| End | -2.058 | -3.018 | -3.018 | 7.68e-11 |

Ordinary solver warnings are not suppressed or treated as proof that a finite
best result is unusable. The actual optimization logs and validation report
retain their counts. The official optimizer's best-result behavior remains in
place; the command adapter separately rejects invalid or excessive commands.

## Read-only reports and Gazebo interfaces

observability.json adds status heartbeat and first-edge/frontier reports in a ninth
official source file, fast_exploration_fsm.cpp. It does not change the original
planning transitions. An empty trigger message is rejected before indexing it.
The adapter waits for a known first edge and an available frontier before sending
one start trigger. UNKNOWN in the start cell is not independently forbidden.

The new depth sensor is physically relocated within the Gazebo model; its reported
optical frame follows the same location. Original physics and payload are retained.
Actual CameraInfo supplies planning intrinsics before the C++ process starts.
The Gazebo adapter converts body-frame measured twists to world axes and converts
PositionCommand to RotorS trajectory messages. It continues existing commands while
the official planner computes the next route. Startup motion is explicitly requested;
no new backward recovery is provided.

Configuration adaptations: max velocity and acceleration 0.6, obstacle inflation
0.35 m, Astar time budget 0.05 s, actual camera intrinsics/FoV, isolated topic names
and writable TSP directory. All other parameters derive from official algorithm.xml.
