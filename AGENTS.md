# Repository Guidelines

## Project Structure & Module Organization

This repository is a ROS 2 Jazzy workspace. ROS packages live under `src/`, grouped by domain: `navigation/` (MINCO, Nav2, communication), `perception/` (Point-LIO, Livox, ROG map, clustering, calibration), `decision/`, `ros_interfaces/`, and `utils/`. Package C++ headers are generally in `include/`, implementations in `src/`, and launch/configuration/assets in `launch/`, `config/`, `maps/`, or `rviz/`. Root `scripts/` contains diagnostics and operations helpers; `docs/` and package `README.md` files hold design notes.

## Build, Test, and Development Commands

Source ROS before building, then run the repository wrapper (it limits parallelism based on memory and builds critical packages serially):

```bash
source /opt/ros/jazzy/setup.bash
./build.bash
```

For a focused build, use `colcon build --symlink-install --packages-select <package>`. Run tests with `colcon test --event-handlers console_cohesion+` and inspect failures with `colcon test-result --verbose`. `./clean.bash` removes generated artifacts. Runtime entry points are ROS launch files, for example `ros2 launch navi2 navigation2.launch.py`; `start.bash` starts the full stack and assumes the expected workspace path and `gnome-terminal`.

## Coding Style & Naming Conventions

C++ follows the checked-in `.clang-format`; format changed C++ with `clang-format` before committing. Use four-space indentation in shell and Python, clear `snake_case` for Python functions/scripts and ROS parameters, and descriptive `PascalCase` types with `camelCase` methods/variables in C++. Keep package names and launch files lowercase with underscores. Preserve existing QoS, frame, topic, and parameter names unless the change updates every publisher, subscriber, and configuration consumer.

## Testing Guidelines

Tests are package-scoped and use the ROS 2 `ament_cmake`/CTest infrastructure where declared (notably `ament_cmake_gtest`). Add tests alongside the affected package and run the focused package test plus `colcon test` when changes cross package boundaries. Name test executables and source files after the behavior under test.

## Commit & Pull Request Guidelines

Work on `develop`; treat `master` as read-only. Follow the documented `<type>: <description>` convention (`feat`, `fix`, `docs`, `style`, `refactor`, `perf`, `test`) and keep messages specific. Pull requests should explain behavior and hardware/configuration impact, link the relevant issue or task, list validation commands, and include launch/RViz screenshots or logs when changing user-visible operation. Call out changes to real-robot IPs, calibration, maps, TFs, or safety behavior explicitly.

## Configuration & Safety

Do not commit secrets or machine-local paths. Review launch files and YAML before running on hardware: this workspace contains real sensor IPs, extrinsics, maps, and static TF assumptions. Verify there is only one publisher for critical transforms and validate topic/QoS changes with all connected nodes.
