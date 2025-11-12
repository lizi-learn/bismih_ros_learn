#include <ros/ros.h>
#include <std_msgs/String.h>

#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_client.hpp>

#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>

using websocketpp::connection_hdl;

class WebsocketRelayNode {
public:
  WebsocketRelayNode(ros::NodeHandle &nh, const std::string &input_topic,
                     const std::string &output_topic, const std::string &uri)
      : nh_(nh), input_topic_(input_topic), output_topic_(output_topic),
        uri_(uri), connected_(false) {
    client_.init_asio();
    client_.clear_access_channels(websocketpp::log::alevel::all);
    client_.set_open_handler([this](connection_hdl hdl) {
      {
        std::lock_guard<std::mutex> lk(mutex_);
        hdl_ = hdl;
        connected_ = true;
      }
      cv_.notify_all();
      ROS_INFO_STREAM("Connected to WebSocket server: " << uri_);
    });
    client_.set_fail_handler([this](connection_hdl) {
      ROS_ERROR_STREAM("Failed to connect to WebSocket server: " << uri_);
    });
    client_.set_close_handler([this](connection_hdl) {
      std::lock_guard<std::mutex> lk(mutex_);
      connected_ = false;
      ROS_WARN("WebSocket connection closed.");
    });
    client_.set_message_handler([this](connection_hdl,
                                       client_t::message_ptr msg) {
      handleIncomingMessage(msg->get_payload());
    });

    ROS_INFO_STREAM("Connecting to WebSocket server: " << uri_);
    websocketpp::lib::error_code ec;
    auto con = client_.get_connection(uri_, ec);
    if (ec) {
      throw std::runtime_error("Could not create connection: " + ec.message());
    }

    client_.start_perpetual();
    client_.connect(con);
    client_thread_ = std::thread([this]() {
      try {
        client_.run();
      } catch (const std::exception &e) {
        ROS_ERROR_STREAM("WebSocket client exception: " << e.what());
      }
    });

    subscriber_ = nh_.subscribe<std_msgs::String>(
        input_topic_, 10,
        std::bind(&WebsocketRelayNode::messageCallback, this,
                  std::placeholders::_1));

    publisher_ =
        nh_.advertise<std_msgs::String>(output_topic_, 10, false);
  }

  ~WebsocketRelayNode() {
    try {
      client_.stop_perpetual();
      if (connected_) {
        websocketpp::lib::error_code ec;
        client_.close(hdl_, websocketpp::close::status::going_away, "", ec);
        if (ec) {
          ROS_WARN_STREAM("Error closing WebSocket connection: " << ec.message());
        }
      }
    } catch (const std::exception &e) {
      ROS_ERROR_STREAM("Exception during WebSocket shutdown: " << e.what());
    }

    if (client_thread_.joinable()) {
      client_thread_.join();
    }
  }

  void spin() {
    ros::spin();
  }

private:
  void messageCallback(const std_msgs::String::ConstPtr &msg) {
    std::unique_lock<std::mutex> lk(mutex_);
    cv_.wait_for(lk, std::chrono::seconds(5),
                 [this]() { return connected_; });

    if (!connected_) {
      ROS_WARN("WebSocket not connected. Message dropped.");
      return;
    }

    websocketpp::lib::error_code ec;
    client_.send(hdl_, msg->data, websocketpp::frame::opcode::text, ec);
    if (ec) {
      ROS_ERROR_STREAM("Failed to send message over WebSocket: " << ec.message());
    } else {
      ROS_INFO_STREAM("Forwarded ROS message to WebSocket: " << msg->data);
    }
  }

  void handleIncomingMessage(const std::string &payload) {
    if (payload.rfind("ACK:", 0) == 0) {
      ROS_DEBUG_STREAM("Received ACK from WebSocket server: " << payload);
      return;
    }

    std_msgs::String ros_msg;
    ros_msg.data = payload;
    publisher_.publish(ros_msg);
    ROS_INFO_STREAM("Published WebSocket message to ROS topic " << output_topic_
                                                                << ": "
                                                                << payload);
  }

  ros::NodeHandle nh_;
  std::string input_topic_;
  std::string output_topic_;
  std::string uri_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;

  typedef websocketpp::client<websocketpp::config::asio_client> client_t;
  typedef client_t::message_ptr message_ptr;
  client_t client_;
  connection_hdl hdl_;
  std::thread client_thread_;

  std::mutex mutex_;
  std::condition_variable cv_;
  bool connected_;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "websocket_relay_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  std::string websocket_uri;
  private_nh.param<std::string>("websocket_uri", websocket_uri,
                                "ws://127.0.0.1:5000/ws");

  try {
    std::string input_topic;
    private_nh.param<std::string>("input_topic", input_topic,
                                  "/websocket_demo/input");

    std::string output_topic;
    private_nh.param<std::string>("output_topic", output_topic,
                                  "/websocket_demo/output");

    WebsocketRelayNode relay(nh, input_topic, output_topic, websocket_uri);
    relay.spin();
  } catch (const std::exception &e) {
    ROS_FATAL_STREAM("Failed to start WebSocket relay node: " << e.what());
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}

