# Differences from official FUEL 662dd23

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

No fixed boundary constraints, position/yaw time stretching, stop-after-each-leg
mode, fallback paths, or unknown-space bypass are included. Additional
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
