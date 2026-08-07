# booster_mujoco

C++ MuJoCo 仿真器，适配 [booster_robotics_sdk](https://github.com/booster-robotics) 的 DDS 接口。
代码框架与 [unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco) 保持一致，仅将
`unitree_sdk2` 的调用替换为 `booster_robotics_sdk`。

## 功能

- 基于 MuJoCo 的机器人实时仿真（内置 Booster B1 系列：T1 23 自由度人形机器人、K1 22 自由度机器人）
- 仿真运行的同时，通过 DDS (FastDDS) 与外部控制器通信：

| 方向 | Topic | 消息类型 | 说明 |
|------|-------|----------|------|
| 订阅 | `rt/joint_ctrl` | `booster_interface::msg::LowCmd` | 关节控制指令（PD 控制） |
| 发布 | `rt/low_state` | `booster_interface::msg::LowState` | 关节状态(q/dq/tau_est) + IMU(rpy/gyro/acc) |
| 发布 | `rt/odom` | `nav_msgs::msg::Odometry` | 基座位姿/速度（等价于 unitree 的 sportmodestate） |
| 发布 | `rt/remote_controller_state` | `booster_interface::msg::RemoteControllerState` | 手柄/遥控器状态 |
| 发布 | `rt/depthimage` | `sensor_msgs::msg::Image` | 深度图（mono8） |

- 支持游戏手柄（xbox / switch / yahboom / betop）模拟遥控器
- 弹性虚拟弹簧带（`enable_elastic_band`），用于吊起人形机器人
- 深度相机噪声/伪影模拟（OpenCV 显示）

## 目录结构

```
booster_mujoco/
├── simulate/
│   ├── CMakeLists.txt
│   ├── config.yaml          # 仿真配置
│   ├── src/
│   │   ├── main.cc
│   │   ├── param.h
│   │   ├── booster_sdk_bridge.h     # DDS 桥（适配 booster_robotics_sdk）
│   │   ├── physics_joystick.h       # 手柄映射
│   │   ├── joystick/                # 手柄驱动 + booster_joystick.h
│   │   ├── mujoco/                  # MuJoCo simulate UI 框架（原样）
│   │   └── utils/                   # depth_display + recurrent_thread
│   └── test/test_booster_sdk.cpp    # DDS 收发测试
├── booster_robots/
│   ├── t1/
│   │   ├── T1_23dof.xml     # T1 机器人模型（含传感器 + 深度相机）
│   │   ├── scene.xml        # 仿真场景
│   │   └── meshes/          # 网格模型
│   └── k1/
│       ├── K1_22dof.xml     # K1 机器人模型（22 自由度，含传感器 + 深度相机）
│       ├── scene.xml        # 仿真场景
│       └── meshes/          # 网格模型
└── fastdds_profile.xml      # DDS 传输配置（UDP + SHM）
```

## 依赖

- MuJoCo（≥ 3.0，`find_package(mujoco)`）
- OpenCV（≥ 4.x）
- GLFW、yaml-cpp、fmt、Boost.Program_options
- [booster_robotics_sdk](https://github.com/booster-robotics)（静态库 `lib/<arch>/libbooster_robotics_sdk.a`，
  默认从同级目录 `../booster_robotics_sdk` 定位，可用 `-DBOOSTER_SDK_DIR=` 覆盖）

## 编译

```bash
cd simulate
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

## 运行

DDS 需要加载 FastDDS 配置文件（UDP 白名单 + SHM 传输）：

```bash
export BOOSTER_FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/fastdds_profile.xml

# 启动仿真（GUI，默认 t1；换 K1：-r k1 或修改 config.yaml 的 robot 字段）
./simulate/build/booster_mujoco

# 启动 DDS 收发测试（订阅 rt/low_state、发布 rt/joint_ctrl，T1/K1 通用）
./simulate/build/test lo
```

命令行参数：`-i <domain_id>`（DDS 域）、`-n <interface>`（网卡）、`-r <robot>`（机器人）、`-s <scene>`。

## 弹性带（Elastic Band）

人形机器人（T1/K1）在无控制器时无法站立，会自然倒下。弹性带通过一个虚拟弹簧-阻尼力
将机器人躯干悬挂在空中，便于调试腿部控制器。

### 启用

在 `simulate/config.yaml` 中设置：

```yaml
enable_elastic_band: 1
```

### 运行中调节（GUI 键盘）

| 按键 | 功能 |
|------|------|
| `7` | 缩短带长（拉力增强，吊得更高） |
| `8` | 加长带长（拉力减弱，吊得更低） |
| `9` | 开关弹性带（enable 切换） |

### 工作原理

弹性带在 `simulate/src/mujoco/simulate.h` 的 `ElasticBand` 类中实现：

- 锚点 `point_ = (0, 0, 1.5)`（世界系上方 1.5m 处）
- 弹簧-阻尼模型：`F = stiffness × (distance − length) − damping × v`，沿锚点方向施加
- 默认 `stiffness_ = 200`、`damping_ = 100`、`length_ = 0`
- 力施加于机器人躯干（`band_attached_link`，自动解析 `Trunk` body），作用于
  `mjData.xfrc_applied`

### 实测效果（rt/odom 基座高度）

| 场景 | 基座高度 | 姿态 |
|---|---|---|
| K1 开启弹性带 | z ≈ 2.04 | 直立（quat_w ≈ 0.996） |
| T1 开启弹性带 | z ≈ 1.44 | 直立（quat_w ≈ 0.998） |
| 关闭弹性带（对照） | z ≈ 0.28 | 倒地 |

## 与 booster_deploy 对接

`booster_deploy`（Python 部署栈）通过 DDS↔ROS2 桥订阅 `rt/low_state`（读取
`motor_state_serial`、`imu_state.rpy/gyro`）并发布 `rt/joint_ctrl`（LowCmd，
`cmd_type=SERIAL`）。本仿真器与该协议完全兼容，可直接在真机控制栈上运行。

## 注意

- 机器人初始位姿为关节零位，人形机器人会自然倒下；接入控制器（如 booster_deploy）
  后由控制器维持平衡。
- 手柄设备默认 `/dev/input/js0`，无手柄时请将 `use_joystick` 设为 0。
- 若关节在无外部程序时仍"自己运动"，通常是残留的 DDS 命令发布进程在向
  `rt/joint_ctrl` 发送指令（如之前测试遗留的 `booster_deploy_cpp`）。用
  `pkill -f "booster_deploy_cp[p]"`（进程名超过 15 字符，`pkill -x` 匹配不到）
  清理后重启仿真即可。
