#ifndef _VM_HPP_
#define _VM_HPP_
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "std_msgs/msg/int32.hpp"
#include "ltlt/dxl.hpp"
#include "opencv2/opencv.hpp"
#include <memory>
#include <functional>
#include <iostream>
#include <queue>
#include <ctime>
#include <unistd.h>
#include <signal.h>
#include <chrono>
using namespace std::chrono_literals;
using std::placeholders::_1;
class Sub : public rclcpp::Node
{
    private:
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr sub_;
        void mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
        std::function<void(const sensor_msgs::msg::CompressedImage::SharedPtr msg)> fn;
        double centermin = 0, centerqbacksave, centerysave = 45, centerymin = 0;
        std::queue<double> centerq, centeryq;
        double gain = 0.32;
        int* p;
        double* c;
        int cnt;
        cv::Mat frame, gray, bin;        
        cv::Mat labels, stats, centroids;        
        int lvel = 0, rvel = 0;
        int cntmin = 0;
        int cx = 0, cy = 0, cw = 0, ch = 0;
        int cameracentroidsx = 320;
        int cameracentroidsy = 45;
        cv::VideoWriter writer1;
        cv::VideoWriter writer2;


        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_;
        std_msgs::msg::Int32 intmsg;
        int vel1, vel2;
        int goal1, goal2;
        void publish_msg();
    public:
        Sub();
        int err;
};
#endif //_SUB_HPP_
