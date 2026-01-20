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
        vision_thread_ = std::thread(&KineVisionNode::run, this);
    }

    void run() {
        cv::namedWindow("kine_vision", cv::WINDOW_AUTOSIZE);

        while (running_) {
            // Wait until a new image arrives or timeout (to keep GUI responsive)
            std::unique_lock lk(mutex_);
            cv_.wait_for(lk, std::chrono::milliseconds(30), [this] {
                return !running_.load(std::memory_order_relaxed) || static_cast<bool>(image_ptr_);
            });

            // Swap shared_ptr out quickly to minimize locked time
            auto img_ptr = std::move(image_ptr_);
            image_ptr_.reset();
            lk.unlock();

            if (img_ptr && !img_ptr->empty()) {
                cv::imshow("kine_vision", *img_ptr);
            }

            const auto key = cv::waitKey(1);
            if (key == 27 || key == 'q') {
                running_ = false;
            }
        }
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }

    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {

        cv::Mat src(msg->height, msg->width, CV_8UC3, msg->data.data());
        cv::Mat bgr;
        cv::cvtColor(src, bgr, cv::COLOR_RGB2BGR);

        auto new_img = std::make_shared<cv::Mat>(std::move(bgr));
        {
            std::lock_guard lk(mutex_);
            image_ptr_ = std::move(new_img);
        }
        cv_.notify_one();
    }

    ~KineVisionNode() override {
        running_ = false;
        cv_.notify_one();
        if (vision_thread_.joinable()) {
            vision_thread_.join();
        }
        cv::destroyAllWindows();
    }

private:
    std::mutex mutex_;
    std::condition_variable cv_;
    std::shared_ptr<cv::Mat> image_ptr_;

    std::thread vision_thread_;
    std::atomic_bool running_{false};

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

int main() {
    rclcpp::init(0, nullptr);

    auto node = std::make_shared<KineVisionNode>();
    rclcpp::spin(node);

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
}
