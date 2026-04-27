# Fork provenance

This is a fork of [`dcmlr/groundgrid`](https://github.com/dcmlr/groundgrid) — the
ROS 2 port of the GroundGrid ground-segmentation algorithm from FU Berlin's
Autonomos-Labs (Steinke et al., IEEE RA-L 2024,
[10.1109/LRA.2023.3333233](https://doi.org/10.1109/LRA.2023.3333233)).

| Field           | Value                                              |
|-----------------|----------------------------------------------------|
| Upstream remote | `https://github.com/dcmlr/groundgrid` (`remote`)   |
| Fork remote     | `git@github.com:jcfurey/groundgrid` (`origin`)     |
| Tracked branch  | `ros2-jazzy` (both)                                |
| Last sync base  | `ab0f093` — *Correct launch file parameter in examples* |

The fork is **upstream-trackable** — every patch lives as a commit on top of
the upstream `ros2-jazzy` history with no rewrites or squashes, so a `git
rebase` against a refreshed upstream is the supported sync path. Verify with:

```bash
git rev-list --count remote/ros2-jazzy..HEAD   # commits ahead of upstream
git rebase remote/ros2-jazzy                   # pick up new upstream work
```

## Local divergence

Three pre-existing commits and one twelve-commit "hardening pass" sit on top
of upstream. Read in the same order they apply:

### Pre-existing fixes (three commits)

These were already on the fork before the hardening pass — workspace-fit
patches needed to make the upstream node build and run on the rovermax
deployment with a non-Velodyne LiDAR.

| Commit    | Subject                                                                          |
|-----------|----------------------------------------------------------------------------------|
| `40af152` | use cloud frame_id instead of hardcoded "velodyne" for TF lookups                |
| `dce55d6` | PointCloud2 row_step mismatch + configurable base_frame                          |
| `3b2eb4b` | C++20 pedantic — remove unused locals, size_t casts, unused-param markers        |

### Hardening pass — 2026-04-27 (twelve commits)

A focused review-and-fix series after the workspace flipped back to RTAB-Map
+ odom EKF (per `project_mapping_vdb_only.md`) but kept GroundGrid as the
default ground-segmentation backend gated by `run_groundgrid`. The review
turned up several classes of issue ranging from cosmetic to ones that would
silently break the node under `rmw_zenoh_cpp` the next time it was enabled.

Ordering is intentional: chores first (no behavior change), then mechanical
refactors, then semantic fixes from least- to most-impactful.

| Commit    | Category   | Subject                                                              |
|-----------|------------|----------------------------------------------------------------------|
| `59f564e` | chore      | drop duplicate includes, empty `onInit`, and dead `seq` counter      |
| `7cc9231` | chore      | clean up build + package metadata (CXX17, drop dead deps, fix typo)  |
| `da38b16` | fix        | correct param descriptor and Velodyne log copy-paste                 |
| `d789ced` | refactor   | drop dead `setLiDAR()` machinery (~80 LoC of vestigial state)        |
| `7c83532` | perf       | hoist per-callback grid_map layer allocation to init (~3 MB/scan)    |
| `92d6fc3` | fix        | rebind grid_map Matrix references per call (drop misleading `static`)|
| `fab3175` | fix        | stop double-counting boundary points across `insert_cloud` chunks    |
| `c1e4900` | fix        | hoist variance computation out of parallel `detect_ground_patches`   |
| `988de84` | fix        | stop blocking the lidar callback on TF lookups (drop 1.1 s timeout)  |
| `10f1bcb` | fix(qos)   | use RELIABLE QoS for odom subscription and grid_map publisher        |
| `798ea08` | fix        | look up PointCloud2 field offsets by name                            |
| `941925d` | fix        | make the world/odom frame name configurable                          |

Each commit message contains the *why* in detail; consult `git log` rather
than restating it here.

## Workspace integration notes

- Wired up via `src/settings/launch/includes/mapping.launch.xml` under the
  `run_groundgrid` env flag. Today off-by-default (the active stack is
  RTAB-Map + odom EKF + RealSense D435i — see `project_mapping_vdb_only.md`).
- Parameters live in `src/settings/params/perception/groundgrid.yaml`,
  including the new `groundgrid/odom_frame` and `groundgrid/base_frame`.
- The QoS fix (`10f1bcb`) is what makes the node usable under the workspace's
  `rmw_zenoh_cpp` middleware — without it the node silently never receives
  odometry and never publishes a usable GridMap. Do not revert it without
  also revisiting the Sierra/EKF publisher QoS.

## Known follow-ups, not addressed

These were called out in the review but explicitly deferred — track if you
re-open the file:

1. **Threading correctness in `insert_cloud`.** Concurrent worker threads
   still write to shared grid cells via non-atomic `+=` updates of points,
   running mean, and Welford `m2`. The chunk-overlap and variance-hoist
   commits closed the obvious races; what remains is real but rare on a
   sparse 363×363 grid. The right fix is per-thread sub-grids or going
   single-threaded — the workspace already runs `max_threads: 2` on the i7
   NUC, so the parallelism is barely earning its keep. Benchmark before
   deciding.
2. **Hardcoded grid resolution / dimension.** `mResolution = 0.33f` and
   `mDimension = 120.0f` are public `const float` members on `GroundGrid`;
   exposing them as ROS parameters is straightforward but the templated
   `block<3,3>`/`block<5,5>` patches in `detect_ground_patch` constrain how
   small the cell-size can sensibly go.
3. **Per-cloud `image_transport::ImageTransport it(shared_from_this())`** in
   `points_callback` — constructed every scan, not cheap. Hoist to a
   deferred-init member.
4. **`tf_broadcaster_` is dataset-only** but allocated unconditionally on the
   node. Cosmetic.

## Sync procedure

```bash
# Inside src/packages/perception/groundgrid:
git fetch remote
git log --oneline remote/ros2-jazzy..HEAD              # what we have on top
git log --oneline HEAD..remote/ros2-jazzy              # what's new upstream
git rebase remote/ros2-jazzy                            # rebase the fork
# Resolve conflicts, then update the "Last sync base" line above.
git push origin ros2-jazzy --force-with-lease           # publish to fork
```

After syncing, update the **Last sync base** line in this file with the new
merge-base commit and a one-line label, and commit alongside the rebase.
