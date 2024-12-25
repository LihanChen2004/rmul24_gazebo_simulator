# rmul24_gazebo_simulator

> **仍在开发中，更新频率较快且不稳定，不考虑向前兼容。请谨慎使用**

## 简介

rmul24_gazebo_simulator 是基于 Gazebo (Ignition 字母版本) 的仿真环境，为 RoboMaster University League 中的机器人算法开发提供仿真环境，方便测试 AI 算法，加快开发效率。

目前 rmul24_gazebo_simulator 提供以下功能：

- RMUL2024、RMUC2024 仿真世界模型

- 网页端局域网联机对战

- 机器人底盘、云台、射击控制

| rmul_2024 | rmuc_2024 |
|:-----------------:|:--------------:|
|![spin_nav.gif](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/spin_nav.1ove3nw63o.gif)|![rmuc_fly.gif](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/rmuc_fly_image.1aoyoashvj.gif)|

## 一. 基本使用

### 1.1 环境配置

- Ubuntu 22.04
- ROS: [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
- Ignition: [Fortress](https://gazebosim.org/docs/fortress/install_ubuntu/)

```sh
cd ~/ros_ws/src

git clone https://github.com/gezp/sdformat_tools.git
git clone https://github.com/SMBU-PolarBear-Robot-Team/rmoss_interfaces.git
git clone https://github.com/SMBU-PolarBear-Robot-Team/rmoss_core.git
git clone https://github.com/SMBU-PolarBear-Robot-Team/rmoss_gazebo.git
git clone https://github.com/SMBU-PolarBear-Robot-Team/rmoss_gz_resources.git --depth=1
git clone https://github.com/SMBU-PolarBear-Robot-Team/rmul24_gazebo_simulator.git
git clone https://github.com/SMBU-PolarBear-Robot-Team/pb2025_robot_description.git

pip install xmacro
```

```sh
cd ..
colcon build --symlink-install
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

### 1.3 切换仿真世界

修改 [gz_world.yaml](./config/gz_world.yaml) 中的 `world`。当前可选: `rmul_2024`, `rmuc_2024`

## 配套导航仿真仓库

- 2025 SMBU PolarBear Sentry Navigation

    [pb2025_sentry_nav](https://github.com/SMBU-PolarBear-Robot-Team/pb2025_sentry_nav.git)

    ![cmu_nav_v1_0](https://raw.githubusercontent.com/LihanChen2004/picx-images-hosting/master/spin_nav.1ove3nw63o.gif)

## 维护者及开源许可证

Maintainer: Lihan Chen, <lihanchen2004@163.com>

rmul24_gazebo_simulator is provided under Apache License 2.0.
