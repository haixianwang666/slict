# SLICT2 适配 Multi-Sensor LiDAR SLAM Challenge 2026

本适配面向激光惯性子赛道，只使用 Airy 点云和内置 IMU。比赛 bag 中的相机
topic 不会被订阅。

## 已适配的数据契约

- 点云 topic：`/rslidar_front_points`，10 Hz；
- IMU topic：`/rslidar_front_imu_data`，约 200 Hz；
- Airy `PointCloud2` 按字段名和运行时 datatype 解码
  `x/y/z/intensity/ring/timestamp/feature`；
- `timestamp` 按官方 `float64` 绝对秒处理，同时保留相对秒/毫秒/微秒/纳秒的
  自动回退，所有点最终转成 SLICT 使用的绝对秒；
- 使用官方 `p_imu = R_imu_lidar p_lidar + t_imu_lidar` 外参，把点云变换到
  内置 IMU/body 坐标；
- 每帧扫描区间从原始 LiDAR `header.stamp` 开始，避免多雷达合并器把下一帧
  起点改成上一帧点时间末尾；
- 结束时在每个原始 LiDAR header 上采样 B 样条并输出 TUM。若回环修改了关键帧，
  对相邻关键帧的 SE(3) 修正做插值后再作用于稠密轨迹；
- TUM 位姿表示 Airy 内置 IMU 原点，四元数顺序为 `qx qy qz qw`。

## 环境与编译

上游 SLICT2 `noetic` 版本要求 Ubuntu 20.04、ROS Noetic、Ceres 2.1、Sophus 和
ROS 1 版本的 `devel_surfel` UFOMap。该分支最新提交已经迁移到 ROS 2，因此必须固定到
下面给出的最后一个 ROS 1/catkin 提交。比赛构建默认关闭 Livox 转换节点，不需要安装
Livox SDK、`livox_ros_driver` 或 `livox_ros_driver2`。

```bash
mkdir -p ~/slict_ws/src
cd ~/slict_ws/src
git clone -b devel_surfel https://github.com/brytsknguyen/ufomap.git
git -C ufomap checkout 1acb0e6a2ba8748dba44229e5541eea199e10b32
ln -s /absolute/path/to/slict2-challenge-2026 slict
cd ~/slict_ws
catkin build slict
source devel/setup.bash
```

若确实需要编译上游 Livox 转换节点，请安装两个 Livox 驱动，并给 catkin 传入
`-DSLICT_BUILD_LIVOX_CONVERTERS=ON`；比赛运行不需要开启此选项。

## 运行两个场景

```bash
roslaunch slict run_challenge_2026.launch \
  bag_file:=/data/sequence_01_0000_0550.bag \
  scene_id:=scene_0001 \
  output_root:=$HOME/slict_challenge_2026 \
  rviz:=false

roslaunch slict run_challenge_2026.launch \
  bag_file:=/data/sequence_02_0560_0970.bag \
  scene_id:=scene_0002 \
  output_root:=$HOME/slict_challenge_2026 \
  rviz:=false
```

bag 播放完约 20 秒后，`autoexit` 会保存轨迹并结束。也可以按 Ctrl-C；退出路径会先
等待处理线程结束再保存。结果位于：

```text
$HOME/slict_challenge_2026/trajectories/scene_0001.txt
$HOME/slict_challenge_2026/trajectories/scene_0002.txt
```

若机器算力不足，先把 `leaf_size` 从 `0.20` 调到 `0.25`，或把
`max_lidar_factor` 从 `8000` 降到 `6000`。不要打开 `ensure_real_time`，离线比赛应优先
保证完整处理。若日志显示静止时原始加速度模长约为 1，本配置会由 SLICT 初始化阶段
自动估计约 9.81 的 `ACC_SCALE`；若已是 m/s²，则估计值应接近 1。

## 校验与打包

```bash
python3 scripts/validate_challenge_tum.py \
  $HOME/slict_challenge_2026/trajectories/scene_0001.txt \
  $HOME/slict_challenge_2026/trajectories/scene_0002.txt

python3 scripts/make_challenge_submission.py \
  --scene-0001 $HOME/slict_challenge_2026/trajectories/scene_0001.txt \
  --scene-0002 $HOME/slict_challenge_2026/trajectories/scene_0002.txt \
  --team YOUR_TEAM_NAME \
  --output $HOME/slict_challenge_2026/submission.zip
```

ZIP 根目录严格为 `README.md` 与 `trajectories/scene_0001.txt`、
`trajectories/scene_0002.txt`。进一步可用官方仓库的 `scoring.validator` 校验。

## 首次真包检查

第一帧日志应包含类似：

```text
Airy PointCloud2 fields=[x,y,z,intensity,ring,timestamp,feature], ...
time=absolute_seconds, span=0.09... s, header_to_first=0.01... s
```

若显示 `relative_auto`，解析器仍会按约 0.1 s 帧长自动选择单位，但应先用
`rosbag info` 和一帧字段统计确认 bag 版本。提交前还应确认每个场景约为 10 Hz 的连续
轨迹、时间戳严格递增、四元数模长接近 1，并分别比较开/关回环的公开榜单指标。
