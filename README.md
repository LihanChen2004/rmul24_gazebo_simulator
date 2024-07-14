# rmul24_gazebo_simulator

[![State-of-the-art Shitcode](https://img.shields.io/static/v1?label=State-of-the-art&message=Shitcode&color=7B5804)](https://github.com/trekhleb/state-of-the-art-shitcode)

> Still in **develop**, shit code

## 简介

rmul24_gazebo_simulator 是基于 Gazebo (Ignition 字母版本) 的仿真环境，为 RoboMaster University League 中的机器人算法开发提供仿真环境，方便测试 AI 算法，加快开发效率。

目前 rmul24_gazebo_simulator 还不完善，仅提供以下功能：

在 rmua19 标准机器人（rmua19_standard_robot）上增加相关传感器，构建不同机器人模型:

- rmua19_standard_robot_a：搭载云台相机 industrial_camera 和搭载激光雷达 rplidar_a2，其中相机放置有在 yaw 轴。

- rmua19_standard_robot_b：搭载云台相机 industrial_camera 和搭载激光雷达 rplidar_a2，其中相机放置有在 pitch 轴。

- rmul24_sentry_robot：搭载云台相机 industrial_camera 和搭载激光雷达 rplidar_a2 和 Livox mid360，其中相机放置有在 pitch 轴

构建 RoboMaster University League 2024 简易场地 (models/RMUL_2024):

- 在 world 中使用 [PerformerDetector](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/performer_detector/PerformerDetector.hh) 插件实现区域 RFID 检测。仍在开发中，尚未与裁判系统对接

## 一. 基本使用

### 1.1 环境配置

- ROS2 版本要求: `Humble`

- Gazebo 仿真器版本要求: `Fortress`

```sh
cd ros_ws/src

git clone https://github.com/LihanChen2004/rmoss_interfaces.git
git clone https://github.com/LihanChen2004/rmoss_core.git
git clone https://github.com/LihanChen2004/rmoss_gazebo.git
git clone https://github.com/LihanChen2004/rmoss_gz_resources.git --depth=1
git clone https://github.com/gezp/sdformat_tools.git
git clone https://github.com/LihanChen2004/rmul24_gazebo_simulator.git

pip install xmacro
```

### 1.2 启动仿真环境

1. 启动仿真环境

    ```sh
    ros2 launch rmul24_gazebo_simulator develop.launch.py
    ```

    **注意：需要点击 Gazebo 左下角橙红色的 `启动` 按钮**

2. 控制机器人移动

    ```sh
    ros2 run rmoss_gz_base test_chassis_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base -p v:=0.3 -p w:=0.3
    #根据提示进行输入，支持平移与自旋
    ```

3. 控制机器人云台

    ```sh
    ros2 run rmoss_gz_base test_gimbal_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base
    #根据提示进行输入，支持绝对角度控制
    ```

4. 机器人射击

    ```sh
    ros2 run rmoss_gz_base test_shoot_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base
    #根据提示进行输入
    ```

5. 运行裁判系统

    ```sh
    ros2 run rmul24_gazebo_simulator simple_competition_1v1.py
    ```

    弹丸伤害为 10，每个机器人HP为 500 ，直到 HP 为 0 时，裁判系统输出胜利者，程序退出。（可重新运行开始）

    通过解析并处理攻击信息 `/referee_system/attack_info` （包括射击者信息以及击中目标信息）实现裁判功能

## TODO

考试月启动... 2024.6.28~7.12

- [ ] mid360 改名为 livox_mid360

- [x] TF Tree 都加上命名空间

- [x] 删除 ignition plugin 中的 `ignition_frame_id`

- [ ] launch 中分离世界生成和机器人生成

## 配套导航仿真仓库

本仿真包可以与 navigation2 结合使用，实现多机器人的导航功能。

详见 [rm_nav_bringup](https://github.com/LihanChen2004/rm_nav_bringup)

## 维护者及开源许可证

Maintainer: Lihan Chen, <lihanchen2004@163.com>

rmul24_gazebo_simulator is provided under Apache License 2.0.
