
#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class KineVisionNode : public rclcpp::Node {

public:
  KineVisionNode() : Node("kine_vision_node") {

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "camera_pixels", 10,
        std::bind(&KineVisionNode::imageCallback, this, std::placeholders::_1));

    running_ = true;
    vision_thread_ = std::thread([this] { run(); });
  }

  void run() {

    cv::namedWindow("kine_vision", cv::WINDOW_AUTOSIZE);

    while (running_) {
      cv::Mat image;
      {
        std::lock_guard lock(mutex_);
        if (!image_.empty()) {
          image = image_.clone();
        }
      }

      if (!image.empty()) {
        cv::imshow("kine_vision", image);
      }

      const auto key = cv::waitKey(1);
      if (key == 27 || key == 'q') {
        running_ = false;
      }
    }
  }

  void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received image with width: %d, height: %d",
                msg->width, msg->height);

    // Convert ROS image message to OpenCV Mat
    cv::Mat img(msg->height, msg->width, CV_8UC3, msg->data.data());
    cv::cvtColor(img, img, cv::COLOR_RGB2BGR);

    std::lock_guard lock(mutex_);
    image_ = img.clone();
  }

  ~KineVisionNode() override {
    running_ = false;
    if (vision_thread_.joinable()) {
      vision_thread_.join();
    }
    cv::destroyAllWindows();
  }

private:
  std::mutex mutex_;
  cv::Mat image_;

  std::thread vision_thread_;
  std::atomic_bool running_{false};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main() {

  rclcpp::init(0, nullptr);

  auto node = std::make_shared<KineVisionNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
