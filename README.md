# rmul24_gazebo_simulator

[![State-of-the-art Shitcode](https://img.shields.io/static/v1?label=State-of-the-art&message=Shitcode&color=7B5804)](https://github.com/trekhleb/state-of-the-art-shitcode)

> **仍在开发中，更新频率较快且不稳定，不考虑向前兼容。请谨慎使用**

## 简介

rmul24_gazebo_simulator 是基于 Gazebo (Ignition 字母版本) 的仿真环境，为 RoboMaster University League 中的机器人算法开发提供仿真环境，方便测试 AI 算法，加快开发效率。

目前 rmul24_gazebo_simulator 还不完善，仅提供以下功能：

在 rmua19 标准机器人（rmua19_standard_robot）上增加相关传感器，构建不同机器人模型:

- rmua19_standard_robot_a：搭载云台相机 industrial_camera 和搭载激光雷达 rplidar_a2，其中相机放置有在 yaw 轴。

- rmua19_standard_robot_b：搭载云台相机 industrial_camera 和搭载激光雷达 rplidar_a2，其中相机放置有在 pitch 轴。

- rmul24_sentry_robot：搭载云台相机 industrial_camera 和搭载激光雷达 rplidar_a2 和 Livox mid360，其中相机放置有在 pitch 轴

构建 RoboMaster University League 2024 简易场地 (models/rmul_2024):

- 在 world 中使用 [PerformerDetector](https://github.com/gazebosim/gz-sim/blob/ign-gazebo6/src/systems/performer_detector/PerformerDetector.hh) 插件实现区域 RFID 检测。仍在开发与裁判系统对接中

构建 RoboMaster University Championship 2024 简易场地 (models/rmuc_2024)

## 一. 基本使用

### 1.1 环境配置

- Ubuntu 22.04
- ROS: [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- Ignition: [Fortress](https://gazebosim.org/docs/fortress/install_ubuntu/)

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

- 启动仿真环境

    ```sh
    ros2 launch rmul24_gazebo_simulator bringup_sim.launch.py
    ```

    **注意：需要点击 Gazebo 左下角橙红色的 `启动` 按钮**

- 控制机器人移动

    ```sh
    ros2 run rmoss_gz_base test_chassis_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base -p v:=0.3 -p w:=0.3
    #根据提示进行输入，支持平移与自旋
    ```

- 键盘控制：

  - 机器人云台

    ```sh
    ros2 run rmoss_gz_base test_gimbal_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base
    #根据提示进行输入，支持绝对角度控制
    ```

  - 机器人射击

    ```sh
    ros2 run rmoss_gz_base test_shoot_cmd.py --ros-args -r __ns:=/red_standard_robot1/robot_base
    #根据提示进行输入
    ```

- 网页端控制

    支持局域网内联机操作，只需要将 localhost 改为主机 ip 即可。

  - 操作手端

    <http://localhost:5000/>

    ```sh
    python3 src/rmul24_gazebo_simulator/scripts/player_web/main_no_vision.py
    ```

  - 裁判系统端

    <http://localhost:2350/>

    ```sh
    python3 src/rmul24_gazebo_simulator/scripts/referee_web/main.py
    ```

## 配套导航仿真仓库

- 与 CMU 导航框架结合使用

    [pb_cmu_navigation](https://github.com/LihanChen2004/pb_cmu_navigation)

    ![cmu_nav_v1_0](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/spin_nav.1ove3nw63o.gif)

- 与 navigation2 导航框架结合使用【停止维护】

    [rm_nav_bringup](https://github.com/LihanChen2004/rm_nav_bringup)

    ![nav_v1_0](https://raw.githubusercontent.com/LihanChen2004/rm_nav_bringup/main/.docs/nav_v1_0.gif)

## 维护者及开源许可证

Maintainer: Lihan Chen, <lihanchen2004@163.com>

rmul24_gazebo_simulator is provided under Apache License 2.0.
