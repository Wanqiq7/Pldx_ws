# 底盘安装雷达 TF 恢复记录

## User Intent

将 Jazzy 导航工作区恢复为雷达固定在底盘的安装关系；保留 base_link_fake 接口。
用户确认沿用旧位置 body→底盘中心 (0,+0.20,0)，yaw_camerainit_to_gimbal 为云台相对底盘角。

## Scope

导航 TF 发布器、两个启动入口、视觉 frame 接入、单雷达内部外参 TF、回归测试与说明。
本次修改涉及比赛验证逻辑，采用最小改动策略。

## Out of Scope

不更换定位、规划、控制算法，不改 topic/QoS，不修改用户已有构建脚本、地图或仿真目录调整。
不运行硬件。视觉俯仰/升降、机械实测标定与无时间戳反馈的延迟补偿另行处理。

## Explorer Findings

### Files inspected

- 原工作区 Point-LIO laserMapping.cpp、mid360.yaml、navigation2.launch.py、sentry1.yaml。
- 当前 fake_vel_transform 源码/头文件/启动/构建定义。
- 当前 bt_manager 启动、main.cpp、tf_utils.cpp、ros_interface.cpp。
- 当前 Point-LIO 单/双雷达启动、mid360.yaml、MPC 速度补偿实现和 sentry1.yaml。
- 视觉 publish2nav.cpp（只读）。

### Active logic path

旧版 base_link 的方向固定在 camera_init，并不代表实体底盘朝向；Jazzy 将该职责交给 base_link_fake。
原 Jazzy 导航启动 fake_vel_transform，用负云台角建立 body→base_link。
BT 启动另外将 vision_gimbal 静态挂在 body 下；BT tf_utils 的 publishDynamicTransform 为空。
单雷达入口 body→livox_frame 使用云台安装偏移与绕 X 轴 180° 四元数。

### Data flow

修改后：重定位发布 map→camera_init；LIO 发布 camera_init→body。
fake_vel_transform 发布静态 body→base_link，以及动态 base_link→base_link_fake。
同一节点接收离线反馈，在收到有限角度时发布 base_link→vision_gimbal。
Nav2/MPC 继续使用 base_link_fake；BT 底盘位置查询继续使用 base_link。

### Risk notes

- 相对角度定义已由用户确认。正号默认逆时针，joint_yaw_sign 可在启动时校准。
- 视觉高度沿用旧版 0.5m，需实测；没有俯仰/升降补偿。
- 离线反馈无采样时间戳，视觉 TF 使用收到消息的时间。实机需核验按观测时间查询的覆盖和延迟。
- 固定安装位置沿用旧版，底盘与 body 零角轴向一致；完整 3D 倾斜下不承诺完全复刻旧版仅 yaw 的位置偏移。
- lidar_roll_offset=0.1745 与旧版一致，本次保留；需结合真实安装检查坡度判断。

### Recommended modification boundary

1. 固定雷达→底盘外参，与云台反馈解耦。
2. 保留虚拟导航 frame，动态反馈只控制视觉云台 frame。
3. 对齐单雷达内部外参 TF；检查速度补偿向量方向。
4. 定向构建、隔离 ROS 域合成输入回归、静态审查。

## Modifier Changes

### Files changed

- src/navigation/fake_vel_transform/src/fake_vel_transform.cpp
- src/navigation/fake_vel_transform/include/fake_vel_transform/fake_vel_transform.hpp
- src/navigation/fake_vel_transform/launch/fake_vel_transform_launch.py
- src/navigation/fake_vel_transform/CMakeLists.txt
- src/navigation/fake_vel_transform/package.xml
- src/navigation/fake_vel_transform/README.md
- src/navigation/fake_vel_transform/test/check_chassis_tf.py
- src/navigation/navi2_bringup/launch/navigation2.launch.py
- src/decision/bt_manager/launch/bt_manager.launch.py
- src/perception/Point-LIO/launch/single_livox_pointlio_intra_process.launch.py
- src/perception/Point-LIO/config/mid360.yaml
- 本记录。

### Key changes

固定外参只在启动时发布至 /tf_static。云台反馈不参与底盘和导航虚拟 frame 的计算。
视觉 frame 的发布职责移至 fake_vel_transform，删除 BT 启动中的静态发布者。
body→livox_frame 与内置 IMU 外参一致：(-0.011,-0.02329,0.04412)，单位旋转。
新增 CTest 合成 ROS 测试，覆盖无云台反馈、多个底盘/云台角组合、旋转偏移和视觉变换。

### Behavior preserved

原导航 frame/topic/QoS、定时频率、LIO 内部外参值和控制器/规划器算法保持。
MPC/planner 的 lidar_offset_y=-0.20 是底盘中心→IMU，与 body→中心 +0.20 相反，正确保留。
BT 旧外参参数保留兼容，但其空 TF 发布函数仍不负责发布；实际参数由 fake_vel_transform 读取。

### Behavior intentionally adjusted

body 与 base_link 刚性连接；云台旋转仅影响 vision_gimbal。
云台 TF 只在收到有效反馈时发布，不使用默认零角伪造反馈，也不在反馈停止后刷新旧角度时间。
安装参数启动时生效；变更后重启发布节点。

### Notes

已安装 navi2/point_lio/bt_manager 的相关 launch 都为源码符号链接，修改直接生效。
执行构建仅限当前 Jazzy 仓库，其 AGENTS.md 允许按项目流程构建；旧参考仓库未修改或构建。

## Auditor Review

### Checks performed

- [x] 关键路径检查
- [x] diff 检查（仅本任务文件）
- [x] grep 检查
- [x] Python AST / YAML 检查
- [x] clang-format --dry-run --Werror
- [x] 定向构建与合成 ROS 测试

构建命令：colcon build --symlink-install --packages-select fake_vel_transform --parallel-workers 1。
构建通过。首次独立回归脚本运行通过。
注册测试：colcon test --packages-select fake_vel_transform --event-handlers console_cohesion+。
最终结果：1 test, 0 errors, 0 failures, 0 skipped；CTest 0.90 秒。
初次 CTest 的断言通过后，ros2 run 包装进程留下子节点导致退出等待；已改为直接启动安装后的可执行文件并保证回收，重跑通过。

### Issues found

没有发现本次变更的静态或合成输入阻塞项。实机定位、速度补偿动态效果和视觉时间对齐尚未验收。
工作区原有大量仿真资源移动/删除及脚本更改，本任务未触碰；执行期间另有地图修改，未纳入本任务。

### Final result

PASS（源码、构建与合成输入验证）；硬件标定及实机验收待执行。
