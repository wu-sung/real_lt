#include "ltlt/sub.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sys/time.h>
#include <signal.h>

using namespace cv;
using namespace std;

bool ctrl_c_pressed = false;
void ctrlc_handler(int) { ctrl_c_pressed = true; }

void Sub::mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (frame.empty()) { 
        std::cerr << "frame empty!" << std::endl; 
        return; 
    }
    
    cv::Mat gray, noise, result;
    cv::Mat labels, stats, centroids;

    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    noise = gray(cv::Rect(0, 270, 640, 90));
    cv::GaussianBlur(noise, noise, cv::Size(5, 5), 0, 0);

    double desiredMean = 60;
    cv::Scalar meanValue = cv::mean(noise);
    double b = desiredMean - meanValue.val[0];
    result = noise + b;
    result = cv::max(0, cv::min(255, result));

    cv::threshold(result, result, 128, 255, cv::THRESH_BINARY);

    int p_width = result.cols;
    int p_height = result.rows;

    int numLabels = cv::connectedComponentsWithStats(result, labels, stats, centroids);
    cv::cvtColor(result, result, cv::COLOR_GRAY2BGR);

    static cv::Point pos((p_width) / 2, p_height / 2);
    static cv::Point poss((p_width) / 2, p_height / 2);
    cv::circle(result, pos, 3, cv::Scalar(0, 255, 0), -1);    

    for (int i = 1; i < numLabels; i++) {
        int left = stats.at<int>(i, cv::CC_STAT_LEFT);
        int top = stats.at<int>(i, cv::CC_STAT_TOP);
        int width = stats.at<int>(i, cv::CC_STAT_WIDTH);
        int height = stats.at<int>(i, cv::CC_STAT_HEIGHT);

        double x = centroids.at<double>(i, 0);
        double y = centroids.at<double>(i, 1);

        cv::Point integer = pos - cv::Point(x, y);
        
        int label = (stats.at<int>(i, cv::CC_STAT_AREA));
        if (label < 80) continue;

        if ((integer.x <= 100 && integer.x >= -100) && ((integer.y >= -80) && (integer.y <= 80))) {
            pos = cv::Point(x, y);
            cv::rectangle(result, cv::Point(left, top), cv::Point(left + width, top + height), cv::Scalar(0, 255, 0), 2);
            cv::circle(result, cv::Point(static_cast<int>(x), static_cast<int>(y)), 3, cv::Scalar(0, 0, 255), -1);
        } else if (integer.y >= 150) {
            pos = cv::Point(x, y);
        } else {
            cv::rectangle(result, cv::Point(left, top), cv::Point(left + width, top + height), cv::Scalar(0, 0, 255), 2);
            cv::circle(result, cv::Point(static_cast<int>(x), static_cast<int>(y)), 3, cv::Scalar(255, 0, 0), -1);
        }
    }

    // centerq.push(pos.x);
    // centeryq.push(pos.y);

    // if (abs(centerq.back() - centerq.front()) > 200) {
    //     centerq.push(centerq.front());
    //     centerq.pop();
    //     centerq.push(centerq.back());
    //     centerq.pop();
    // }
    // if (abs(centeryq.back() - centeryq.front()) > 60) {
    //     centeryq.push(centeryq.front());
    //     centeryq.pop();
    //     centeryq.push(centeryq.back());
    //     centeryq.pop();
    // }
    // if(abs(err - (cameracentroidsx - centerq.back()))>100){
    //     centerq.push(centerq.front());
    //     centerq.pop();
    //     centerq.push(centerq.back());
    //     centerq.pop();
    // }

    centerqbacksave = centerq.back();
    centerysave = centeryq.back();

    cv::circle(result, cv::Point(centerqbacksave, centerysave), 2, cv::Scalar(0, 0, 255), 2);
    cv::rectangle(result, cv::Rect(cx, cy, cw, ch), cv::Scalar(0, 0, 255), 2);
    
    err = cameracentroidsx - centerq.back();
    centerq.pop();
    centeryq.pop();

    writer1 << frame;
    writer2 << result;
    cv::imshow("frame", frame);
    cv::imshow("result", result);
    cv::waitKey(1);
    RCLCPP_INFO(this->get_logger(), "Received Image : %s,%d,%d", msg->format.c_str(), frame.rows, frame.cols);
}

void Sub::publish_msg()
{
    intmsg.data = err;
    RCLCPP_INFO(this->get_logger(), "Publish: %d", intmsg.data);
    pub_->publish(intmsg);
}

Sub::Sub() : Node("camsub_wsl")
{
    centerq.push(320);
    centeryq.push(45);
    writer1.open("output1.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30, cv::Size(640, 360));
    writer2.open("output2.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30, cv::Size(640, 90));
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile, std::bind(&Sub::mysub_callback, this, _1));
    pub_ = this->create_publisher<std_msgs::msg::Int32>("err", qos_profile);
    timer_ = this->create_wall_timer(50ms, std::bind(&Sub::publish_msg, this));
}