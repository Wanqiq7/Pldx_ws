# Ubuntu 24.04 / ROS 2 Jazzy 迁移记录

## User Intent

将工程默认环境从 Ubuntu 22.04 / ROS 2 Humble 调整为 Ubuntu 24.04 / ROS 2 Jazzy。

## Scope

仅更新环境入口、脚本提示和文档中的发行版默认值；保留 ROS_DISTRO 覆盖机制。

## Out of Scope

不修改 topic、frame、参数、比赛策略、ROS 包接口或构建产物。

## Explorer Findings

### Files inspected

README.md、build.bash、scripts/monitor_odom_rogmap.py、scripts/savemap.bash，以及相关工具 README 和 Livox 集成文档；源码 CMake/package.xml 未发现发行版硬编码。

### Active logic path

build.bash 和 savemap.bash 是发现的实际 ROS 环境入口；其他命中位于错误提示或文档示例。

### Data flow

ROS 环境通过 `/opt/ros/<ROS_DISTRO>/setup.bash` 注入，运行时 topic、节点和参数路径不变。

### Risk notes

Jazzy 与 Humble 的二进制依赖可用性仍需在目标电脑上验证；本次未执行构建。

### Recommended modification boundary

只改上述环境入口与文档文件。

## Modifier Changes

### Files changed

README.md、build.bash、scripts/monitor_odom_rogmap.py、scripts/savemap.bash、src/utils/pcd2pgm/README.md、src/perception/msg_convert/README.md、src/perception/livox_ros_driver2/INTERNAL_LIDAR_MERGE.md。

### Key changes

默认 ROS_DISTRO 改为 jazzy，系统和文档链接改为 Ubuntu 24.04 / ROS 2 Jazzy，脚本保留环境变量覆盖。

### Behavior preserved

节点、话题、QoS、参数、行为树和比赛策略未改动。

### Behavior intentionally adjusted

仅调整默认 ROS 环境路径与对应说明。

## Auditor Review

### Checks performed

- [x] 关键路径检查
- [x] diff 检查
- [x] grep 检查
- [x] 文档与脚本静态检查
- [x] 未执行构建（未获用户许可）

### Issues found

未发现越界修改。

### Final result

PASS
