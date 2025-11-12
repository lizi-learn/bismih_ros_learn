#!/usr/bin/env python3
"""
Simple Flask WebSocket server that echoes messages received from the ROS client.
"""

import logging
import os
import threading

from collections import deque
from datetime import datetime

from flask import Flask, jsonify, Response, request
from flask_sock import Sock

app = Flask(__name__)
sock = Sock(app)

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("cloud_server")

MESSAGES = deque(maxlen=100)
OUTBOUND_QUEUE = deque()
LOCK = threading.Lock()

HTML_PAGE = """<!DOCTYPE html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <title>ROS WebSocket Demo Dashboard</title>
  <style>
    body { font-family: Arial, sans-serif; max-width: 720px; margin: 2rem auto; }
    h1 { color: #333; }
    form { margin-bottom: 1.5rem; }
    #status { margin-bottom: 1rem; color: #555; }
    #messages { list-style: none; padding: 0; }
    #messages li { border-bottom: 1px solid #ddd; padding: 0.75rem 0; }
    .timestamp { color: #888; font-size: 0.85rem; }
    .payload { font-weight: bold; }
    .direction { font-size: 0.85rem; color: #0066cc; }
    .client { font-size: 0.8rem; color: #999; }
    button { padding: 0.5rem 1rem; margin-bottom: 1.5rem; }
    input[type="text"] { width: 70%; padding: 0.5rem; margin-right: 0.5rem; }
    .error { color: #cc0000; }
    .success { color: #008000; }
  </style>
</head>
<body>
  <h1>ROS WebSocket Demo</h1>
  <p id="status">正在加载...</p>
  <form id="publish-form">
    <input type="text" id="publish-input" placeholder="输入要发送给 ROS 的消息" required />
    <button type="submit">发送</button>
    <span id="publish-result"></span>
  </form>
  <button id="refresh">手动刷新</button>
  <button id="trigger-default">发送默认 ROS 测试消息</button>
  <ul id="messages"></ul>
  <script>
    async function fetchMessages() {
      try {
        const resp = await fetch('/messages');
        if (!resp.ok) {
          throw new Error('网络异常');
        }
        const data = await resp.json();

        const status = document.getElementById('status');
        const list = document.getElementById('messages');
        list.innerHTML = '';

        if (data.messages.length === 0) {
          status.textContent = '尚未收到任何 ROS 消息。';
          return;
        }

        status.textContent = `最新消息数量：${data.messages.length}（最多展示最近 100 条）`;
        data.messages.forEach((item) => {
          const li = document.createElement('li');
          li.innerHTML = `
            <div class="direction">${item.direction === 'from_ros' ? '← 来自 ROS' : '→ 云端发送至 ROS'}</div>
            <div class="timestamp">${item.timestamp}</div>
            <div class="payload">消息：${item.payload}</div>
            ${item.ack ? `<div>ACK：${item.ack}</div>` : ''}
            ${item.client ? `<div class="client">客户端：${item.client}</div>` : ''}
          `;
          list.appendChild(li);
        });
      } catch (error) {
        document.getElementById('status').textContent = '刷新失败：' + error.message;
      }
    }

    async function publishMessage(event, preset) {
      if (event) {
        event.preventDefault();
      }
      const input = document.getElementById('publish-input');
      const result = document.getElementById('publish-result');
      const value = preset !== undefined ? preset : input.value.trim();
      if (!value) {
        result.textContent = '请输入消息';
        result.className = 'error';
        return;
      }
      try {
        const resp = await fetch('/publish', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify({ message: value })
        });
        const data = await resp.json();
        if (!resp.ok) {
          throw new Error(data.error || '发布失败');
        }
        result.textContent = '已发送';
        result.className = 'success';
        if (preset === undefined) {
          input.value = '';
        }
        fetchMessages();
      } catch (error) {
        result.textContent = error.message;
        result.className = 'error';
      }
    }

    document.getElementById('refresh').addEventListener('click', fetchMessages);
    document.getElementById('publish-form').addEventListener('submit', publishMessage);
    document.getElementById('trigger-default').addEventListener('click', () =>
      publishMessage(null, "hello from ROS")
    );
    setInterval(fetchMessages, 2000);
    fetchMessages();
  </script>
</body>
</html>
"""


@app.route("/", methods=["GET"])
def index():
    """Dashboard page."""
    return Response(HTML_PAGE, mimetype="text/html")


@app.route("/health", methods=["GET"])
def health():
    """HTTP health check endpoint."""
    return jsonify({"status": "ok"})


@app.route("/messages", methods=["GET"])
def list_messages():
    """Return recent messages for the dashboard."""
    with LOCK:
        data = list(MESSAGES)
    return jsonify(
        {"messages": data, "count": len(data), "server_time": datetime.utcnow().isoformat() + "Z"}
    )


@app.route("/publish", methods=["POST"])
def publish_message():
    """Queue a message to be sent to ROS clients over WebSocket."""
    payload = request.get_json(silent=True) or {}
    message = (payload.get("message") or "").strip()

    if not message:
        return jsonify({"error": "message 字段不能为空"}), 400

    item = {
        "timestamp": datetime.utcnow().isoformat() + "Z",
        "payload": message,
        "ack": None,
        "direction": "from_cloud",
        "client": "dashboard",
    }
    with LOCK:
        OUTBOUND_QUEUE.append(item)
    logger.info("Queued outbound message to ROS: %s", message)
    return jsonify({"status": "queued", "message": message})


@sock.route("/ws")
def websocket_route(ws):
    """
    Basic echo websocket route.

    The ROS node sends text frames. The server logs the payload and replies
    with a confirmation message to demonstrate bidirectional communication.
    """
    client_addr = ws.environ.get("REMOTE_ADDR")
    logger.info("WebSocket connected from %s", client_addr)
    try:
        while True:
            try:
                data = ws.receive(timeout=1)
            except TimeoutError:
                data = None

            outbound_item = None
            with LOCK:
                if OUTBOUND_QUEUE:
                    outbound_item = OUTBOUND_QUEUE.popleft()

            if outbound_item:
                ws.send(outbound_item["payload"])
                logger.info("Sent outbound payload to ROS: %s", outbound_item["payload"])
                with LOCK:
                    MESSAGES.appendleft(outbound_item)

            if data is None:
                continue

            logger.info("Received payload: %s", data)
            ack_message = f"ACK:{data}"
            item = {
                "timestamp": datetime.utcnow().isoformat() + "Z",
                "payload": data,
                "ack": ack_message,
                "direction": "from_ros",
                "client": client_addr,
            }
            with LOCK:
                MESSAGES.appendleft(item)
            ws.send(ack_message)
    except Exception as exc:  # pylint: disable=broad-except
        logger.exception("WebSocket error: %s", exc)


if __name__ == "__main__":
    host = os.environ.get("FLASK_RUN_HOST", "0.0.0.0")
    port = int(os.environ.get("FLASK_RUN_PORT", "5000"))
    app.run(host=host, port=port)

