# ROS Noetic Topic Demo（C++）

本示例在 `/home/pc/learn_ros1/ros_neotic_topic_demo` 下提供一个最小的 ROS Noetic 话题通信演示，包含发布者 `talker` 与订阅者 `listener`。

## 环境准备

1. 确保已安装 ROS Noetic 并完成初始化（`rosdep`, `roscore` 等）。
2. 打开终端并执行：
   ```bash
   source /opt/ros/noetic/setup.bash
   cd /home/pc/learn_ros1/ros_neotic_topic_demo
   catkin_make
   source devel/setup.bash
   ```

## 运行示例

1. 启动 ROS 核心：
   ```bash
   roscore
   ```
2. 在新的终端中启动发布者节点：
   ```bash
   source /opt/ros/noetic/setup.bash
   source /home/pc/learn_ros1/ros_neotic_topic_demo/devel/setup.bash
   rosrun topic_demo talker
   ```
3. 再开一个终端启动订阅者节点：
   ```bash
   source /opt/ros/noetic/setup.bash
   source /home/pc/learn_ros1/ros_neotic_topic_demo/devel/setup.bash
   rosrun topic_demo listener
   ```

`talker` 会在 `chatter` 话题上发布字符串消息，`listener` 将打印收到的内容。

## 节点说明

- `talker`: 每秒发布一次 `std_msgs/String` 消息，内容包含一个递增计数。
- `listener`: 订阅 `chatter` 话题并在终端输出接收到的消息。

## 后续扩展建议

- 使用自定义消息类型替换 `std_msgs/String`。
- 为节点添加参数化话题名与频率设置。
- 编写对应的单元测试或集成测试脚本。

