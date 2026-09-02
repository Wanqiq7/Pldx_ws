# Simulation TF and BT Startup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Eliminate the simulation-only `camera_init` Message Filter drops and the invalid `nav2_constrained_smoother` BT plugin load, while leaving all files outside `src/simulation` untouched and bringing the default Nav2 behavior tree fully active.

**Architecture:** Keep Point-LIO's registered cloud in its published `camera_init` frame throughout the Nav2 observation-source contract, instead of overriding its sensor origin with the physical `front_mid360` frame and introducing an unnecessary dynamic-TF dependency. Let Nav2 Jazzy load its built-in BT node libraries by omitting `plugin_lib_names`, and provide the recovery action servers referenced by Jazzy's installed default behavior tree.

**Tech Stack:** ROS 2 Jazzy, Nav2 1.3.12, Gazebo Sim 8, Point-LIO, Python `unittest`/PyYAML, Bash runtime smoke test, `ament_cmake_pytest`.

---

## Scope And Diagnosis

Only these repository paths may be created or modified:

- `src/simulation/simulator/rmu_gazebo_simulator/config/simulation_nav2_params.yaml`
- `src/simulation/simulator/rmu_gazebo_simulator/test/test_simulation_map_config.py`
- `src/simulation/simulator/rmu_gazebo_simulator/test/check_sim_navigation_runtime.sh`
- `src/simulation/plans/2026-08-25-simulation-tf-bt-startup.md`

Do not modify Point-LIO, Nav2, `navi2_bringup`, or any installed file under `/opt/ros/jazzy`.

Observed baseline from `ros2 launch rmu_gazebo_simulator simulation.launch.py use_rviz:=false`:

- The global costmap repeatedly drops `/cloud_registered` messages in `camera_init` as older than the TF cache.
- `/cloud_registered -> map` is immediately transformable through the static `camera_init -> odom -> map` chain.
- The configured `sensor_frame: front_mid360` additionally requires the dynamic `odom -> base_link` chain. At cloud arrival this transform commonly trails the cloud stamp by 5-25 ms, causing the Message Filter failure path.
- With only `sensor_frame` changed to `camera_init` in a temporary parameter file, a 45-second run produced zero matching Message Filter warnings.
- `/opt/ros/jazzy/lib/libnav2_constrained_smoother.so` exports a `nav2_core::Smoother` plugin and does not export `BT_RegisterNodesFromPlugin`; it is not a BT plugin.
- Omitting `plugin_lib_names` removes the library error. Setting it to YAML `[]` is invalid here and crashes with `No parameter value set`, so the key must be absent.
- After that fix, the installed default BT still requires `/spin` and `/backup`, while the simulation currently starts only `Wait`. Enabling `Spin`, `BackUp`, and `Wait` made both `/behavior_server` and `/bt_navigator` reach `active [3]`.

Before committing, resolve the branch-policy mismatch with the repository owner. The current checkout is on `master`, no local or remote `develop` ref exists, and the only similarly named ref is `origin/develop_reset`. The worktree also contains unrelated dirty/untracked work. Do not guess that `develop_reset` replaces `develop`, switch branches, or clean the tree destructively; implementation and verification may proceed with exact-path staging, but the commit steps below require an approved non-`master` branch.

### Task 1: Add Red Configuration Regression Tests

**Files:**

- Modify: `src/simulation/simulator/rmu_gazebo_simulator/test/test_simulation_map_config.py:21-65`
- Test: `src/simulation/simulator/rmu_gazebo_simulator/test/test_simulation_map_config.py`

- [ ] **Step 1: Add a shared Nav2 parameter loader**

Insert below `PACKAGE_ROOT`:

```python
def load_nav2_params():
    path = PACKAGE_ROOT / "config" / "simulation_nav2_params.yaml"
    with path.open() as stream:
        return yaml.safe_load(stream)
```

Update `test_launch_map_argument_is_not_shadowed_by_empty_node_parameter()` to call `load_nav2_params()` instead of opening the file inline.

- [ ] **Step 2: Add tests for the cloud frame and BT library contract**

Add these methods to `SimulationMapConfigTest`:

```python
    def test_registered_cloud_uses_its_published_frame_as_sensor_frame(self):
        nav2_params = load_nav2_params()
        source = nav2_params["global_costmap"]["global_costmap"][
            "ros__parameters"
        ]["obstacle_layer"]["mid360"]

        self.assertEqual(source["topic"], "/cloud_registered")
        self.assertEqual(source["sensor_frame"], "camera_init")

    def test_bt_navigator_uses_jazzy_builtin_plugins(self):
        nav2_params = load_nav2_params()
        bt_params = nav2_params["bt_navigator"]["ros__parameters"]

        self.assertNotIn("plugin_lib_names", bt_params)
```

- [ ] **Step 3: Add a test for default-tree recovery servers**

Add this method to the same class:

```python
    def test_default_bt_recovery_action_servers_are_configured(self):
        nav2_params = load_nav2_params()
        behavior_params = nav2_params["behavior_server"]["ros__parameters"]
        expected_plugins = {
            "spin": "nav2_behaviors::Spin",
            "backup": "nav2_behaviors::BackUp",
            "wait": "nav2_behaviors::Wait",
        }

        self.assertEqual(
            set(behavior_params["behavior_plugins"]), set(expected_plugins)
        )
        for name, plugin_type in expected_plugins.items():
            self.assertEqual(behavior_params[name]["plugin"], plugin_type)
```

- [ ] **Step 4: Run the focused test and verify the three new assertions fail**

Run:

```bash
python3 -m pytest \
  src/simulation/simulator/rmu_gazebo_simulator/test/test_simulation_map_config.py \
  -q
```

Expected: the two existing tests pass and the three new tests fail because `sensor_frame` is `front_mid360`, `plugin_lib_names` is present, and only `wait` is configured.

- [ ] **Step 5: Commit the red tests after the branch prerequisite is resolved**

```bash
git add src/simulation/simulator/rmu_gazebo_simulator/test/test_simulation_map_config.py
git commit -m "test: cover simulation tf and bt contracts"
```

### Task 2: Correct The Simulation Nav2 Contract

**Files:**

- Modify: `src/simulation/simulator/rmu_gazebo_simulator/config/simulation_nav2_params.yaml:9-30`
- Modify: `src/simulation/simulator/rmu_gazebo_simulator/config/simulation_nav2_params.yaml:219-224`
- Modify: `src/simulation/simulator/rmu_gazebo_simulator/config/simulation_nav2_params.yaml:553-568`
- Test: `src/simulation/simulator/rmu_gazebo_simulator/test/test_simulation_map_config.py`

- [ ] **Step 1: Remove the invalid custom BT library parameter**

Delete the two misleading comments and the parameter itself:

```yaml
    # Jazzy requires this parameter to be present. This non-BT Nav2 library
    # is a harmless loadable placeholder; built-in BT plugins load automatically.
    plugin_lib_names: [nav2_constrained_smoother]
```

Keep `use_sim_time: True` as the first actual parameter under `bt_navigator.ros__parameters`. Do not replace the deleted value with `plugin_lib_names: []`; an untyped empty YAML array is rejected by this Jazzy parameter path.

- [ ] **Step 2: Make the observation source match the Point-LIO message frame**

Change the global costmap source to:

```yaml
        mid360:
          data_type: PointCloud2
          topic: /cloud_registered
          # Point-LIO has already registered this cloud in camera_init.
          sensor_frame: camera_init
```

Do not change `/cloud_registered` timestamps, the adapter TF cache duration, `transform_tolerance`, or Point-LIO source code. The runtime probe showed the static `camera_init -> map` path is valid; the incorrect frame override is the source of the extra dynamic dependency.

- [ ] **Step 3: Provide the recovery actions used by Jazzy's default BT**

Replace the behavior plugin block with:

```yaml
    behavior_plugins: ["spin", "backup", "wait"]
    spin:
      plugin: "nav2_behaviors::Spin"
    backup:
      plugin: "nav2_behaviors::BackUp"
    wait:
      plugin: "nav2_behaviors::Wait"
```

Remove the stale commented custom `BackUpTwzFree` block so the active simulation contract is unambiguous. Do not add `DriveOnHeading` or `AssistedTeleop`; the installed default `navigate_to_pose_w_replanning_and_recovery.xml` used here references only `Spin`, `BackUp`, and `Wait` recovery action servers.

- [ ] **Step 4: Run the focused test and verify it is green**

Run:

```bash
python3 -m pytest \
  src/simulation/simulator/rmu_gazebo_simulator/test/test_simulation_map_config.py \
  -q
```

Expected: `5 passed`.

- [ ] **Step 5: Commit the configuration fix after the branch prerequisite is resolved**

```bash
git add \
  src/simulation/simulator/rmu_gazebo_simulator/config/simulation_nav2_params.yaml
git commit -m "fix: align simulation tf and nav2 plugins"
```

### Task 3: Add An Agent-Runnable Runtime Smoke Test

**Files:**

- Create: `src/simulation/simulator/rmu_gazebo_simulator/test/check_sim_navigation_runtime.sh`

- [ ] **Step 1: Create the runtime checker**

Create the executable script with this complete content:

```bash
#!/usr/bin/env bash
set -euo pipefail

log_file="$(mktemp /tmp/rmu_sim_navigation.XXXXXX.log)"
launch_pid=""

cleanup() {
    if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
        kill -INT "${launch_pid}"
        wait "${launch_pid}" || true
    fi
}
trap cleanup EXIT

ros2 launch rmu_gazebo_simulator simulation.launch.py use_rviz:=false \
    >"${log_file}" 2>&1 &
launch_pid="$!"

deadline=$((SECONDS + 35))
while (( SECONDS < deadline )); do
    bt_state="$(ROS2CLI_NO_DAEMON=1 ros2 lifecycle get /bt_navigator 2>/dev/null || true)"
    behavior_state="$(ROS2CLI_NO_DAEMON=1 ros2 lifecycle get /behavior_server 2>/dev/null || true)"
    if [[ "${bt_state}" == "active [3]" && "${behavior_state}" == "active [3]" ]]; then
        break
    fi
    sleep 1
done

if [[ "${bt_state:-}" != "active [3]" || "${behavior_state:-}" != "active [3]" ]]; then
    tail -n 120 "${log_file}"
    echo "Nav2 lifecycle activation failed; full log: ${log_file}" >&2
    exit 1
fi

ros2 run sim_nav_adapter check_sim_tf.py --ros-args -p timeout_sec:=10.0
sleep 30

failure_pattern='BT_RegisterNodesFromPlugin|libnav2_constrained_smoother.so|Message Filter dropping message: frame '\''camera_init'\''.*earlier than all the data in the transform cache'
if rg -n "${failure_pattern}" "${log_file}"; then
    echo "Simulation startup regression detected; full log: ${log_file}" >&2
    exit 1
fi

echo "PASS: simulation TF and Nav2 startup contracts held for 30 seconds"
rm -f "${log_file}"
```

- [ ] **Step 2: Make the checker executable**

Run:

```bash
chmod +x src/simulation/simulator/rmu_gazebo_simulator/test/check_sim_navigation_runtime.sh
```

- [ ] **Step 3: Validate the checker syntax before building**

Run:

```bash
bash -n src/simulation/simulator/rmu_gazebo_simulator/test/check_sim_navigation_runtime.sh
```

Expected: exit status 0 and no output. The runtime execution happens after the focused build in Task 4.

- [ ] **Step 4: Commit the runtime checker after the branch prerequisite is resolved**

```bash
git add src/simulation/simulator/rmu_gazebo_simulator/test/check_sim_navigation_runtime.sh
git commit -m "test: add simulation navigation smoke check"
```

### Task 4: Build And Verify The Focused Package

**Files:**

- Verify: `src/simulation/simulator/rmu_gazebo_simulator/config/simulation_nav2_params.yaml`
- Verify: `src/simulation/simulator/rmu_gazebo_simulator/test/test_simulation_map_config.py`
- Verify: `src/simulation/simulator/rmu_gazebo_simulator/test/check_sim_navigation_runtime.sh`

- [ ] **Step 1: Build the simulator package with tests enabled**

```bash
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-select rmu_gazebo_simulator \
  --cmake-args -DBUILD_TESTING=ON
```

Expected: `rmu_gazebo_simulator` finishes successfully.

- [ ] **Step 2: Run its package tests**

```bash
source install/setup.bash
colcon test --packages-select rmu_gazebo_simulator \
  --event-handlers console_cohesion+
colcon test-result --verbose
```

Expected: zero failed tests and `test_simulation_map_config` passes.

- [ ] **Step 3: Run the full runtime smoke test**

```bash
src/simulation/simulator/rmu_gazebo_simulator/test/check_sim_navigation_runtime.sh
```

Expected: both lifecycle nodes become active, the TF checker passes, and neither original log signature appears during the 30-second observation window.

- [ ] **Step 4: Confirm the edit boundary**

```bash
git diff --name-only -- src/simulation
git diff --name-only -- . ':(exclude)src/simulation/**'
```

Expected: the first command lists only the scoped files above; the second command contains no changes introduced by this implementation. Existing unrelated user changes may still appear and must not be modified or staged.

- [ ] **Step 5: Record final verification in the PR description**

Use these exact validation entries:

```text
- python3 -m pytest src/simulation/simulator/rmu_gazebo_simulator/test/test_simulation_map_config.py -q
- colcon test --packages-select rmu_gazebo_simulator --event-handlers console_cohesion+
- src/simulation/simulator/rmu_gazebo_simulator/test/check_sim_navigation_runtime.sh
```

Call out that the change affects simulation frames and Nav2 recovery behavior only; it does not alter real-robot TFs, sensor extrinsics, maps, IP addresses, or QoS.
