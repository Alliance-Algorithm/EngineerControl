#include <cstdio>
#include <memory>
#include <opencv2/videoio.hpp>
#include <rclcpp/qos.hpp>
#include <thread>

#include <eigen3/Eigen/Dense>

#include <hikcamera/image_capturer.hpp>

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/detail/marker__struct.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "attitude_algorithm.hpp"
#include "util/fps_counter.hpp"

namespace EngineerVisual {
class CameraNode : public rclcpp::Node {

public:
  CameraNode(const std::string &node_name)
      : Node(node_name), tail_attitude_algorithm_(),
        thread_(&CameraNode::thread_main, this) {
    tail_attitude_publisher_ = create_publisher<geometry_msgs::msg::Pose>(
        "/engineer/tail/attitude/pose", rclcpp::QoS(1));
    marker_publisher_ = create_publisher<visualization_msgs::msg::Marker>(
        "/engineer/tail/attitude/marker", rclcpp::QoS(1));
  }

private:
  void thread_main() {

    FpsCounter fps_counter;

    hikcamera::ImageCapturer::CameraProfile camera_profile;
    {
      using namespace std::chrono_literals;
      camera_profile.exposure_time = 3ms;
      camera_profile.gain = 16.9807;
      camera_profile.invert_image = true;
    }
    hikcamera::ImageCapturer image_capturer(camera_profile);

    while (rclcpp::ok()) {
      if (fps_counter.count())
        RCLCPP_INFO(this->get_logger(), "fps: %d ", fps_counter.get_fps());

      geometry_msgs::msg::Pose msg;
      visualization_msgs::msg::Marker ore_marker;

      auto image = image_capturer.read();

      tail_attitude_algorithm_.Calculate(image);

      auto rotate = tail_attitude_algorithm_.Rotate();
      auto position = tail_attitude_algorithm_.Position();
      ore_marker.header.frame_id = "engineer";
      ore_marker.type = visualization_msgs::msg::Marker::SPHERE;
      ore_marker.action = visualization_msgs::msg::Marker::ADD;
      ore_marker.scale.x = ore_marker.scale.y = ore_marker.scale.z = 0.05;
      ore_marker.color.r = 1.0;
      ore_marker.color.g = 0.0;
      ore_marker.color.b = 0.0;
      ore_marker.color.a = 1.0;
      ore_marker.lifetime = rclcpp::Duration::from_seconds(0.1);
      ore_marker.header.stamp = now();
      ore_marker.pose.position.x = position.x();
      ore_marker.pose.position.y = position.y();
      ore_marker.pose.position.z = position.z();
      marker_publisher_->publish(ore_marker);

      msg.position.x = position.x();
      msg.position.y = position.y();
      msg.position.z = position.z();
      msg.orientation.set__w(rotate.w());
      msg.orientation.set__x(rotate.x());
      msg.orientation.set__y(rotate.y());
      msg.orientation.set__z(rotate.z());
      tail_attitude_publisher_->publish(msg);
    }
  }

  AttitudeAlgorithm tail_attitude_algorithm_;

  rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr
      tail_attitude_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr
      marker_publisher_;

  std::thread thread_;
};

} // namespace EngineerVisual

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<EngineerVisual::CameraNode>("engineer_visual");
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();

  // Test
  // hikcamera::ImageCapturer hikCapture;
  // auto rawImg = hikCapture.read();

  // cv::imshow("raw", rawImg);

  // cv::waitKey(0); // 这句确保窗口一直打开
  // TestEnd

  printf("hello world visual_exchange package\n");
  return 0;
}
