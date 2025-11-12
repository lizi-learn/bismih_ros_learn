# ROS Noetic Service Demo

本示例为一个简单的 ROS Noetic 服务端与客户端项目，演示如何在 C++ 中定义并调用自定义服务。包含以下组件：

- `ros_noetic_service_demo/srv/AddTwoInts.srv`：定义请求两个整型并返回其和的服务接口。
- `ros_noetic_service_demo/src/add_two_ints_server.cpp`：服务端节点，接收请求后计算并返回结果。
- `ros_noetic_service_demo/src/add_two_ints_client.cpp`：客户端节点，通过命令行传参调用服务。

## 构建步骤

```bash
cd /home/pc/ros_learn
source /opt/ros/noetic/setup.bash
catkin_make
```

## 运行服务与客户端

```bash
source devel/setup.bash
rosrun ros_noetic_service_demo add_two_ints_server
# 新终端中运行
rosrun ros_noetic_service_demo add_two_ints_client 2 3
```

运行后，客户端会调用服务端计算 2 与 3 的和，并在日志中输出结果。

