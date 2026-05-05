# ros2_moveit

基于 ROS 2 + MoveIt 2 的 6 自由度机械臂运动规划项目，支持仿真与真实机器人（达妙电机 CAN 总线驱动）。

---
另一个开源项目：本项目算是个子项目，其中使用到的hardware和urdf模型是复用的这个项目：https://github.com/aa483680845-dev/Open_Arm_Mini.git 。
sw文件和详细的说明以及需要的依赖也在这个项目中。 

moveit2安装：https://moveit.ai/install-moveit2/binary/

## 项目结构

```
src/
├── robot_description/      # 机器人 URDF 与网格模型
├── robot_config/           # MoveIt 配置（SRDF、运动学、控制器、关节限位）
├── moveit_interfaces/      # 运动规划接口节点（位姿规划 & 笛卡尔路径规划）
├── my_robot_hardware/      # ros2_control 硬件接口（CAN 总线 → 达妙电机）
├── robot_dm_driver/        # 达妙电机底层 CAN 驱动库：https://gitee.com/kit-miao/dm-tools/tree/master/DM_DeviceSDK
└── tf2_listener/           # 末端执行器 TF 实时监听节点,打印末端相当于坐标
```
##视频演示

https://github.com/user-attachments/assets/d7efbc5a-303b-4419-b8f6-11ab1bbc370e






## 各包说明

### robot_description
机器人本体描述，由 SolidWorks 导出。

- URDF：`base_link` → `link_1` → ... → `link_6`，共 6 个旋转关节（`joint_1` ~ `joint_6`）
- 包含各连杆碰撞、惯性、网格（STL）信息

### robot_config
MoveIt 2 配置包，由 [MoveIt Setup Assistant](https://moveit.picknik.ai/main/doc/examples/setup_assistant/setup_assistant_tutorial.html#moveit-setup-assistant) 生成。

| 文件 | 内容 |
|------|------|
| `robot_1.srdf` | 规划组 `arm`、预设位姿（home / pose_1 / pose_2）、碰撞矩阵 |
| `kinematics.yaml` | 运动学求解器：KDL |
| `joint_limits.yaml` | 各关节速度/加速度限制 |
| `moveit_controllers.yaml` | MoveIt 控制器配置 |
| `pilz_cartesian_limits.yaml` | Pilz 规划器笛卡尔空间速度/加速度限制 |

moveit设置助手完成了大部分的工作，可以说使用完成MoveIt Setup Assistant配置，就可以使用模拟组件规划机械臂了，
如果先要控制真实的硬件，需要自己写hardware interface插件，我写的插件在`my_robot_hardware`包中。

### moveit_interfaces
运动规划接口节点，通过 launch 参数 `use_cartesian` 切换两种规划模式：

**模式一：目标位姿规划**（`use_cartesian:=false`，默认）

节点：`moveit_interface_main`，配置文件：`config/target_pose.yaml`

- 从参数读取目标位置（x/y/z）和姿态（roll/pitch/yaw）
- 支持绝对姿态（`use_relative: false`）和相对末端自身轴的增量姿态（`use_relative: true`）
- 调用 MoveIt OMPL 规划器规划关节空间路径
- 在 RViz 中可视化目标点与规划轨迹，手动确认后执行

**模式二：[笛卡尔路径规划](https://moveit.picknik.ai/main/doc/examples/move_group_interface/move_group_interface_tutorial.html#move-group-c-interface)**（`use_cartesian:=true`）

节点：`moveit_cartesian`，配置文件：`config/cartesian_waypoints.yaml`

- 从参数读取多组相对位移（dx/dy/dz）与姿态偏转（droll/dpitch/dyaw）
- 以当前末端位姿为起点，逐点累加，最后自动返回起点
- 调用 `computeCartesianPath` 进行笛卡尔直线插补规划（插值步长 `eef_step` 可配）
- 在 RViz 中用绿色路径线 + 坐标轴标注可视化所有路径点

**启动命令：**
```bash
# 先启动,加载move_group + RViz + 控制器管理器 + 控制器
ros2 launch moveit_interfaces moveit_main.launch.py
# 位姿规划
ros2 launch moveit_interfaces moveit_main.launch.py
# 笛卡尔路径规划
ros2 launch moveit_interfaces moveit_main.launch.py use_cartesian:=true
```

### my_robot_hardware
ros2_control `SystemInterface` 硬件接口，对接真实机器人。

- 通过 USB2CANFD 与 6 个达妙电机通信
- 采用 **MIT 模式**（位置 + 速度 + 力矩混合控制），各关节 kp/kd 参数从 URDF `<param>` 读取
- 使用 **Pinocchio** 进行动力学计算（重力补偿等）
- 电机 CAN ID：`0x01` ~ `0x06`，对应 `joint_1` ~ `joint_6`

### robot_dm_driver
达妙（Damiao）电机底层 CAN 协议驱动库。

- 提供电机注册、使能/失能、MIT 模式指令发送、状态回调解析
- 状态量（位置/速度/力矩）读写采用互斥锁保护，线程安全

### tf2_listener
末端执行器位姿实时监听节点。

- 以 10 Hz 频率查询 `base_link` → `link_6` 的 TF 变换
- 将末端位置（xyz）和姿态（四元数）打印到终端，用于调试与状态监控
---

##未来更新
   moveit2内容十分庞大，目前的项目仅用到了其movegroup-C++ API进行简单的轨迹规划和笛卡尔空间规划。
   未来我会更新C++ API，使用其他的规划算法, 加入视觉识别。
