#include "ltlt/sub.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sys/time.h>
#include <signal.h>

using namespace cv;
using namespace std;

bool ctrl_c_pressed = false;
void ctrlc_handler(int) { ctrl_c_pressed = true; }

// CompressedImage 메시지를 처리하는 콜백 함수
void Sub::mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    // 압축된 이미지 데이터를 디코딩하여 frame에 저장
    frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (frame.empty()) { 
        std::cerr << "frame empty!" << std::endl; 
        return; 
    }
    
    // 이미지 처리에 사용할 변수들 선언
    cv::Mat gray, noise, result;
    cv::Mat labels, stats, centroids;

    // 이미지를 그레이스케일로 변환
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    
    // 이미지의 일부 영역을 선택하여 노이즈 제거
    noise = gray(cv::Rect(0, 270, 640, 90));
    cv::GaussianBlur(noise, noise, cv::Size(5, 5), 0, 0);

    // 밝기 조정을 위해 평균값 계산 및 조정
    double desiredMean = 60;
    cv::Scalar meanValue = cv::mean(noise);
    double b = desiredMean - meanValue.val[0];
    result = noise + b;
    result = cv::max(0, cv::min(255, result));

    // 이진화 처리
    cv::threshold(result, result, 128, 255, cv::THRESH_BINARY);

    // 결과 이미지의 너비와 높이
    int p_width = result.cols;
    int p_height = result.rows;

    // 연결된 구성 요소 분석
    int numLabels = cv::connectedComponentsWithStats(result, labels, stats, centroids);
    cv::cvtColor(result, result, cv::COLOR_GRAY2BGR);

    // 초기 위치 설정
    static cv::Point pos((p_width) / 2, p_height / 2);
    static cv::Point poss((p_width) / 2, p_height / 2);
    cv::circle(result, pos, 3, cv::Scalar(0, 255, 0), -1);    

    // 각 라벨에 대해 처리
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


    // 현재 큐의 끝값 저장
    centerqbacksave = centerq.back();
    centerysave = centeryq.back();

    // 결과 이미지에 원과 사각형 그리기
    cv::circle(result, cv::Point(centerqbacksave, centerysave), 2, cv::Scalar(0, 0, 255), 2);
    cv::rectangle(result, cv::Rect(cx, cy, cw, ch), cv::Scalar(0, 0, 255), 2);
    
    // 에러 값 계산
    err = cameracentroidsx - centerq.back();
    centerq.pop();
    centeryq.pop();

    // 비디오 작성기 객체에 프레임과 결과 저장
    writer1 << frame;
    writer2 << result;
    cv::imshow("frame", frame);
    cv::imshow("result", result);
    cv::waitKey(1);
    RCLCPP_INFO(this->get_logger(), "Received Image : %s,%d,%d", msg->format.c_str(), frame.rows, frame.cols);
}

// 에러 값을 퍼블리시하는 함수
void Sub::publish_msg()
{
    intmsg.data = err;
    RCLCPP_INFO(this->get_logger(), "Publish: %d", intmsg.data);
    pub_->publish(intmsg);
}

// Sub 클래스의 생성자: ROS2 노드 "camsub_wsl"을 초기화
Sub::Sub() : Node("camsub_wsl")
{
    centerq.push(320);
    centeryq.push(45);
    
    // 비디오 작성기 초기화
    writer1.open("output1.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30, cv::Size(640, 360));
    writer2.open("output2.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30, cv::Size(640, 90));
    
    // QoS 프로파일 설정 (베스트 에포트 모드)
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    
    // "image/compressed" 토픽에 대한 서브스크립션 생성
    sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile, std::bind(&Sub::mysub_callback, this, _1));
    
    // "err" 토픽에 대한 퍼블리셔 생성
    pub_ = this->create_publisher<std_msgs::msg::Int32>("err", qos_profile);
    
    // 50ms마다 publish_msg 함수를 호출하는 타이머 생성
    timer_ = this->create_wall_timer(50ms, std::bind(&Sub::publish_msg, this));
}
