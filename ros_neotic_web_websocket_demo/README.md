# ROS WebSocket Demo

本示例展示了 ROS Noetic C++ 节点与云端 Flask WebSocket 服务之间的通信流程。

## 目录结构

- `src/ros_websocket_demo/`：ROS 包源码
  - `src/websocket_relay_node.cpp`：订阅 ROS 话题并通过 WebSocket 转发消息的 C++ 节点
  - `scripts/cloud_server.py`：基于 Flask + Flask-Sock 的云端 WebSocket 服务

## 环境依赖

1. ROS Noetic（Ubuntu 20.04）
2. 系统依赖：
   ```bash
   sudo apt update
   sudo apt install ros-noetic-ros-base \
                    ros-noetic-std-msgs \
                    ros-noetic-roscpp \
                    libwebsocketpp-dev \
                    libssl-dev \
                    libboost-system-dev \
                    libboost-thread-dev
   ```
3. Python 依赖（建议置于虚拟环境）：
   ```bash
   python3 -m pip install --upgrade pip
   pip install flask flask-sock gunicorn
   ```

## 构建 ROS 包

```bash
cd /home/pc/learn_ros1/ros_neotic_web_websocket_demo
catkin_make
source devel/setup.bash
```

## 运行步骤

1. 启动 Flask WebSocket 服务：
   ```bash
   cd /home/pc/learn_ros1/ros_neotic_web_websocket_demo/src/ros_websocket_demo
   FLASK_RUN_HOST=0.0.0.0 FLASK_RUN_PORT=5000 python3 scripts/cloud_server.py
   ```
   服务启动后将：
   - 在 `http://160.202.240.232:5000` 提供自动刷新的可视化页面（每 2 秒更新一次），可实时查看最近 100 条 ROS 消息，并允许云端向 ROS 发送消息；
   - 在 `http://160.202.240.232:5000/messages` 暴露消息 JSON；
   - POST `{"message": "内容"}` 至 `http://160.202.240.232:5000/publish` 可从云端推送消息至 ROS；
   - WebSocket 入口为 `ws://0.0.0.0:5000/ws`，供 ROS 节点收发消息。

2. 在新的终端中加载工作空间环境：
   ```bash
   cd /home/pc/learn_ros1/ros_neotic_web_websocket_demo
   source devel/setup.bash
   ```

3. 启动 WebSocket 转发节点：
   ```bash
   rosrun ros_websocket_demo websocket_relay_node _websocket_uri:=ws://160.202.240.232:5000/ws
   ```
   - 也可通过 `_input_topic:=/your/topic` 指定订阅的 ROS 话题（默认 `/websocket_demo/input`）。
   - 通过 `_output_topic:=/another/topic` 指定云端消息在 ROS 端发布的目标话题（默认 `/websocket_demo/output`）。

4. 在另一个终端发布测试消息：
   ```bash
   rostopic pub /websocket_demo/input std_msgs/String "data: 'hello from ROS'"
   ```

5. 浏览器访问 `http://160.202.240.232:5000` 可实时查看最新消息（包含方向、时间、客户端与 ACK）并通过输入框向 ROS 发送消息；同时在 Flask 服务终端查看日志。
   - 页面提供“发送默认 ROS 测试消息”按钮，快速向 `/websocket_demo/input` 推送 `hello from ROS`。
   - 健康检查接口位于 `http://160.202.240.232:5000/health`。

6. 云端页面发送的消息会通过 WebSocket 推送至 ROS 节点，并发布到 `_output_topic`（默认 `/websocket_demo/output`），可以在本地测试：
   ```bash
   rostopic echo /websocket_demo/output
   ```

## 关闭

- 停止 ROS 节点（`Ctrl+C`）
- 停止 Flask 服务（`Ctrl+C`）

## 进一步扩展

- 在 Flask 端解析消息并持久化或转发至其他服务
- 在 ROS 端进一步封装响应逻辑，将 ACK 转换成 ROS 话题或服务

