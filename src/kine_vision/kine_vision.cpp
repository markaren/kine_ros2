
#include <opencv2/opencv.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class KineVisionNode : public rclcpp::Node {
public:
    KineVisionNode() : Node("kine_vision_node") {
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera_pixels", 10,
            std::bind(&KineVisionNode::imageCallback, this, std::placeholders::_1));

        goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);

        running_ = true;
        vision_thread_ = std::thread(&KineVisionNode::run, this);
    }

    std::optional<cv::Point> detectSphere(const cv::Mat& img) {
        if (img.empty()) return std::nullopt;

        cv::Mat gray;
        if (img.channels() == 3 || img.channels() == 4) {
            cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = img.clone();
        }

        // Reduce noise while keeping edges for Hough
        cv::medianBlur(gray, gray, 5);

        // Try HoughCircles first (fast and robust for circular shapes)
        std::vector<cv::Vec3f> circles;
        double dp = 1.2;
        double minDist = std::max(10.0, static_cast<double>(img.rows) / 8.0);
        double param1 = 100.0; // higher threshold for Canny (internal)
        double param2 = 30.0;  // accumulator threshold for circle centers
        int minRadius = 8;
        int maxRadius = 0; // 0 = no explicit max

        cv::HoughCircles(gray, circles, cv::HOUGH_GRADIENT, dp, minDist, param1, param2, minRadius, maxRadius);
        if (!circles.empty()) {
            const auto &c = circles.front();
            return cv::Point(cvRound(c[0]), cvRound(c[1]));
        }

        // Fallback: edge -> contours -> pick most circular large contour
        cv::Mat edges;
        cv::Canny(gray, edges, 100, 200);
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(edges, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

        double bestScore = 0.0;
        std::optional<cv::Point> bestCenter;
        for (const auto &cnt : contours) {
            double area = cv::contourArea(cnt);
            if (area < 50.0) continue; // ignore tiny contours

            cv::Point2f center;
            float radius = 0.0f;
            cv::minEnclosingCircle(cnt, center, radius);
            if (radius < 5.0f) continue;

            double circleArea = CV_PI * radius * radius;
            double circularity = area / circleArea; // close to 1.0 for circular shapes
            // Score: prefer larger, more circular contours
            double score = circularity * area;
            if (score > bestScore && circularity > 0.5) {
                bestScore = score;
                bestCenter = cv::Point(cvRound(center.x), cvRound(center.y));
            }
        }

        return bestCenter;
    }

    void run() {
        const std::string winname = "kine_vision";
        cv::namedWindow(winname, cv::WINDOW_AUTOSIZE);

        while (running_) {
            // Wait until a new image arrives or timeout (to keep GUI responsive)
            std::unique_lock lk(mutex_);
            cv_.wait_for(lk, std::chrono::milliseconds(30), [this] {
                return !running_ || static_cast<bool>(image_ptr_);
            });

            // Swap shared_ptr out quickly to minimize locked time
            auto img_ptr = std::move(image_ptr_);
            image_ptr_.reset();
            lk.unlock();

            if (img_ptr && !img_ptr->empty()) {
                const auto point = detectSphere(*img_ptr);
                if (point) {
                    //publish goal

                    geometry_msgs::msg::PoseStamped pose;
                    pose.header.stamp = this->now();
                    pose.header.frame_id = "world";
                    pose.pose.position.x = point->x;
                    pose.pose.position.y = point->y;
                    // goal_pose_pub_->publish(pose);

                    //draw point
                    cv::Mat img_copy = *img_ptr;
                    cv::circle(img_copy, *point, 5, cv::Scalar(0, 255, 0), -1);
                    cv::imshow(winname, img_copy);
                }

                cv::imshow(winname, *img_ptr);
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

        const auto new_img = std::make_shared<cv::Mat>(std::move(bgr));
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
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;

};

int main() {
    rclcpp::init(0, nullptr);

    auto node = std::make_shared<KineVisionNode>();
    rclcpp::spin(node);

    if (rclcpp::ok()) {
        rclcpp::shutdown();
    }
}
