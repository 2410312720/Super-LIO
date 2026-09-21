<div align="center">
  <h1>⚡Super-LIO</h1>
  <h2>Super-LIO: A Robust and Efficient LiDAR-Inertial Odometry System with a Compact Mapping Strategy</h2>
  <p><strong>This work has been accepted to <i> IEEE Robotics and Automation Letters (RA-L 2026)</i>.</strong></p>
  <br>

  [![Code](https://img.shields.io/badge/GitHub-181717?style=flat-square&logo=github&logoWidth=16)](https://github.com/Liansheng-Wang/Super-LIO.git) [![arXiv](https://img.shields.io/badge/arXiv-blue?logo=arxiv&color=%23B31B1B)](https://arxiv.org/abs/2509.05723) [![IEEE](https://img.shields.io/badge/RAL2026-004088.svg)](https://ieeexplore.ieee.org/document/11347459) [![Bilibili](https://img.shields.io/badge/Bilibili-00A1D6?style=flat-square&logo=bilibili&logoColor=white&logoWidth=16)](https://www.bilibili.com/video/BV11wBeBYEp6) [![YouTube](https://img.shields.io/badge/YouTube-FF0000?style=flat-square&logo=youtube&logoColor=white&logoWidth=16)](https://youtu.be/m9-hl8s5DDw)
</div>


<div align="center">
  <p>
    <a href="https://github.com/Liansheng-Wang/Super-LIO/tree/ros1" style="text-decoration: none;">
      <img src="https://img.shields.io/badge/🔄 SWITCH - ROS1 Noetic-3b82f6?style=for-the-badge&logo=ros&logoColor=white&logoWidth=22" alt="Switch to ROS1"
      onmouseover="this.src='https://img.shields.io/badge/🔄 SWITCH - ROS1 Noetic-1e40af?style=for-the-badge&logo=ros&logoColor=white&logoWidth=22'"
      onmouseout="this.src='https://img.shields.io/badge/🔄 SWITCH - ROS1 Noetic-3b82f6?style=for-the-badge&logo=ros&logoColor=white&logoWidth=22'"/>
    </a>&nbsp;&nbsp;
    <a href="https://github.com/Liansheng-Wang/Super-LIO/tree/ros2" style="text-decoration: none;">
      <img src="https://img.shields.io/badge/✅ ACTIVE - ROS2 Humble/Iron/Jazzy-22c55e?style=for-the-badge&logo=ros&logoColor=white&logoWidth=22" alt="ROS2 Active"
      onmouseover="this.src='https://img.shields.io/badge/✅ ACTIVE - ROS2 Humble/Iron/Jazzy-166534?style=for-the-badge&logo=ros&logoColor=white&logoWidth=22'"
      onmouseout="this.src='https://img.shields.io/badge/✅ ACTIVE - ROS2 Humble/Iron/Jazzy-22c55e?style=for-the-badge&logo=ros&logoColor=white&logoWidth=22'"/>
    </a>&nbsp;&nbsp;
    <a href="#" style="text-decoration: none; cursor: default;">
      <img src="https://img.shields.io/badge/🖥️ PLATFORM - X86 + ARM64-8b5cf6?style=for-the-badge&logo=linux&logoColor=white&logoWidth=22" alt="X86 and ARM Support"
      onmouseover="this.src='https://img.shields.io/badge/🖥️ PLATFORM - X86_64 + ARM64-4f46e5?style=for-the-badge&logo=linux&logoColor=white&logoWidth=22'"
      onmouseout="this.src='https://img.shields.io/badge/🖥️ PLATFORM - X86_64 + ARM64-8b5cf6?style=for-the-badge&logo=linux&logoColor=white&logoWidth=22'"/>
    </a>
  </p>
</div>

## Overview

<p align="center">
  <img src="docs/system_overview.png" width="95%">
</p>

**Key Features:  Efficient · Robust · Cross-Platform Compatible · Supports Both ROS1/ROS2 Versions**

Super-LIO is a robust and efficient LiDAR–Inertial Odometry (LIO) system designed for real-time and large-scale autonomous navigation. It introduces a compact and structured mapping strategy that enables predictable correspondence search and stable state estimation. The system is validated through extensive real-world experiments and comparisons with state-of-the-art methods, which demonstrates that Super-LIO not only achieves **excellent accuracy** but also maintains **lower resource consumption** and realizes a nearly **1.2–4× higher real-time processing speed**⚡.


**Contributors**: [Liansheng Wang](https://github.com/Liansheng-Wang), [Xinke Zhang](https://github.com/PSQzzzxk), [Chenhui Li](https://github.com/kermitLHH), [Dongjiao He](https://github.com/Joanna-HE), [Yihan pan](https://github.com/pyh3552), Jianjun Yi.


## Quickly Run

**For ROS1 Users**: Please switch to the **ros1** branch and follow the instructions at [ros1 branch](https://github.com/Liansheng-Wang/Super-LIO/tree/ros1)

### Requirements

Ubuntu 24(22).04 · C++20 · ROS Jazzy(Humble) · Eigen · PCL 

### Dependencies

glog · TBB

```bash
sudo apt install libgoogle-glog-dev libtbb-dev
```

### Build & Run
```bash
git clone https://github.com/Liansheng-Wang/Super-LIO.git
cd Super-LIO
colcon build

source install/setup.bash
ros2 launch super_lio Livox_mid360.py

```

#### 🔁 Relocalization Mode
Super-LIO supports relocalization using a pre-built map, allowing the system to resume localization from a saved map without restarting the mapping process.
This mode is useful for long-term deployment, repeated missions, or recovery after tracking loss.

Before running relocalization, please make sure that:
- A map has been previously saved to disk.

```bash
cd PATH_2_Super-LIO
source install/setup.bash
ros2 launch super_lio relocation.py
```


## Datasets
<p align="center">
  <img src="docs/datasets_compressed.png" width="95%">
</p>

Super-LIO is evaluated on multiple real-world datasets covering diverse environments,
including indoor, outdoor, and large-scale scenes.

> **TODO**: Dataset download links and detailed descriptions will be provided in the future.


---

## Publications

If your like our projects, please cite us and support us with a star 🌟.
We kindly recommend to cite [our paper](https://ieeexplore.ieee.org/document/11347459) if you find this library useful:

```latex
@article{wang2026superlio,
  title   = {Super-LIO: A Robust and Efficient LiDAR-Inertial Odometry System with a Compact Mapping Strategy},
  author  = {Wang, Liansheng and Zhang, Xinke and Li, Chenhui and He, Dongjiao and Pan, Yihan and Yi, Jianjun},
  journal = {IEEE Robotics and Automation Letters},
  year    = {2026},
  volume  = {11},
  number  = {3},
  pages   = {2666--2673},
  doi     = {10.1109/LRA.2026.3653372}
}
```


## Update Logs

<details>
<summary>Click to expand <b>Update Logs</b> (click to collapse)</summary>

<br>

- 2026-01-04  
  - Separate ROS interface and algorithm.
  - Refactor SuperLIOReLoc to inherit from SuperLIO.
  - Code style aligned with ROS2 version.

- 2026-01-04
  - The main branch is renamed to ros1
  - add ros2 branch

- 2026-01-04 21:51
  - release ROS2 version

</details>




```mermaid
flowchart TD
    A[节点启动<br/>Px4TrajNode / Px4TrajMode 构造] --> B[声明参数<br/>trajectory_file / clicked_path_file / use_clicked_path / 速度高度等]
    B --> C[创建发布器与订阅器<br/>planned_path / trajectory_preview / clicked_path / active_point]
    C --> D[创建 clicked_point 订阅]
    D --> E[创建 preview_timer]
    E --> F[初始化预览缓存<br/>clearPreviewPathCache + rebuildPreviewPathCache]

    F --> G{模式是否被激活?}
    G -- 否 --> H[等待]
    G -- 是 --> I[onActivate]

    I --> J[loadTrajectoryFromFile]
    J --> K[读取 csv 航点<br/>loadWaypointsFromFile]
    K --> L[计算 Bezier 平滑轨迹<br/>buildBezierTrajectorySamples]
    L --> M[缓存预览路径消息<br/>cachePreviewPath]
    M --> N{轨迹是否有效?}
    N -- 否 --> O[进入 Idle<br/>保持当前位置]
    N -- 是 --> P[state = GoToStart]
    P --> Q[发布 planned_path]
    Q --> R[如果 use_clicked_path=true<br/>设置首次点击清空标志]

    R --> S[循环执行 updateSetpoint]
    S --> T{当前位置与轨迹是否有效?}
    T -- 否 --> U[goto_setpoint 保持悬停]
    T -- 是 --> V{当前状态}

    V -- GoToStart --> W[飞向轨迹起点]
    W --> X{到达起点?}
    X -- 否 --> S
    X -- 是 --> Y[切到 HoldAtStart]

    V -- HoldAtStart --> Z[起点悬停 start_hold_time]
    Z --> AA{悬停时间到?}
    AA -- 否 --> S
    AA -- 是 --> AB[切到 ExecuteTrajectory]

    V -- ExecuteTrajectory --> AC[按时间计算目标距离]
    AC --> AD[插值得到目标位置]
    AD --> AE[计算目标速度/航向]
    AE --> AF[trajectory_setpoint 更新]
    AF --> AG{是否到达轨迹末尾?}
    AG -- 否 --> S
    AG -- 是 --> AH[切到 HoldAtEnd]

    V -- HoldAtEnd --> AI[终点悬停]
    AI --> S

    H --> AJ[previewTimerCallback]
    AJ --> AK[直接发布缓存的 preview_path_msg]
    AK --> H

    D --> AL[clickedPointCallback]
    AL --> AM{clear_clicked_path_on_next_click?}
    AM -- 是 --> AN[清空 clicked_waypoints_enu_]
    AM -- 否 --> AO[保留已有点击点]
    AN --> AP[加入新点击点]
    AO --> AP
    AP --> AQ[写入 clicked_path.csv]
    AQ --> AR[发布 clicked_path]
    AR --> AS{use_clicked_path?}
    AS -- 否 --> H
    AS -- 是 --> AT[rebuildPreviewPathCache]
    AT --> AU[重新读取 clicked_path.csv]
    AU --> AV[重新计算 Bezier 预览]
    AV --> AW[更新 preview_path_msg]
    AW --> H
```