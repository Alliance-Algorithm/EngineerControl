#include <chrono>
#include <cstddef>
#include <cstdio>
#include <memory>
#include <opencv2/videoio.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <thread>

#include <eigen3/Eigen/Dense>

#include <hikcamera/image_capturer.hpp>

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <unistd.h>
#include <visualization_msgs/msg/detail/marker__struct.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace EngineerVisual {
class WatchDogs : public rclcpp::Node {

public:
  WatchDogs(const std::string &node_name)
      : Node(node_name), timer(std::time(NULL)),
        thread_(&WatchDogs::thread_main, this) {
    watch_topic = create_subscription<geometry_msgs::msg::Pose>(
        "/engineer/axis/big_yaw", rclcpp::QoS(1),
        std::bind(&WatchDogs::feed_dog, this, std::placeholders::_1));
  }

private:
  void thread_main() {

    hikcamera::ImageCapturer::CameraProfile camera_profile;
    {
      using namespace std::chrono_literals;
      camera_profile.exposure_time = 3ms;
      camera_profile.gain = 16.9807;
      camera_profile.invert_image = true;
    }
    hikcamera::ImageCapturer image_capturer(camera_profile);
    thread_t = std::thread(&WatchDogs::thread_temp, this);
    while (rclcpp::ok()) {
      if (time(NULL) - timer > 5) {
        RCLCPP_FATAL(this->get_logger(), "Unity Stopped: serial");
        throw;
      }
    }
  }

  void thread_temp() { system("ros2 launch rmcs_bringup yuheng.launch.py"); }
  void feed_dog(const geometry_msgs::msg::Pose::SharedPtr) {
    timer = time(NULL);
  }

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr watch_topic;

  std::thread thread_;
  std::thread thread_t;
  time_t timer;
};

} // namespace EngineerVisual

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor executor;
  auto node = std::make_shared<EngineerVisual::WatchDogs>("engineer_visual");
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
