#include "ltlt/pub.hpp"

// 생성자: ROS2 노드 "campub"을 초기화
Pub::Pub() : Node("campub")
{
    // QoS 프로파일 설정 (최근 10개의 메시지만 유지)
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    
    // "image/compressed" 토픽에 대한 퍼블리셔 생성
    pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("image/compressed", qos_profile);
    
    // 25ms마다 publish_msg 함수를 호출하는 타이머 생성
    timer_ = this->create_wall_timer(25ms, std::bind(&Pub::publish_msg, this));
    
    // GStreamer 파이프라인을 사용하여 비디오 캡처 객체 초기화
    if (!cap.open(src, cv::CAP_GSTREAMER)) {
        // 비디오 캡처 초기화 실패 시 에러 로그 출력 후 노드 종료
        RCLCPP_ERROR(this->get_logger(), "Could not open video using GStreamer pipeline!");
        rclcpp::shutdown();
        return;
    }

    // 다이나믹셀 초기화
    if(!dxl.open())
    {
        // 다이나믹셀 초기화 실패 시 에러 로그 출력 후 노드 종료
        RCLCPP_ERROR(this->get_logger(), "dynamixel open error");
        rclcpp::shutdown();
        return;
    } 

    // 콜백 함수 생성 및 바인딩
    std::function<void(const std_msgs::msg::Int32::SharedPtr msg)> fn;
    fn = std::bind(&Pub::mysub_callback, this, dxl, _1);
    
    // "err" 토픽에 대한 서브스크립션 생성
    sub_ = this->create_subscription<std_msgs::msg::Int32>("err", qos_profile, fn);
}

// 주기적으로 호출되어 비디오 프레임을 퍼블리싱하는 함수
void Pub::publish_msg()
{
    // 비디오 프레임을 읽어옴
    cap >> frame;
    if (frame.empty()) { 
        // 프레임이 비어있으면 에러 로그 출력
        RCLCPP_ERROR(this->get_logger(), "frame empty"); 
        return;
    }
    // 프레임을 압축 이미지 메시지로 변환하여 퍼블리싱
    msg = cv_bridge::CvImage(hdr, "bgr8", frame).toCompressedImageMsg();
    pub_->publish(*msg);
}

// "err" 토픽으로부터 메시지를 수신했을 때 호출되는 콜백 함수
void Pub::mysub_callback(Dxl& mdxl, const std_msgs::msg::Int32::SharedPtr intmsg)
{
    // 수신된 에러 값
    err = intmsg->data;
    
    // 에러 값에 기반하여 왼쪽과 오른쪽 바퀴 속도 계산
    lvel = 100 - gain * err; // 왼쪽 바퀴 속도
    rvel = -(100 + gain * err); // 오른쪽 바퀴 속도
    
    // 속도 값을 로그로 출력
    RCLCPP_INFO(this->get_logger(), "Received message: %d %d", lvel, rvel);
    
    // 다이나믹셀 속도 설정
    mdxl.setVelocity(lvel, rvel);
}
