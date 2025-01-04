#include "rclcpp/rclcpp.hpp"
#include "opencv4/opencv2/opencv.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/msg/bool.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <memory>

class Answer : public rclcpp::Node
{
public:
    Answer();
    typedef struct ColorRange{
        int low_H = 0;
        int low_S = 0;
        int low_V = 0;
        int high_H = 0;
        int high_S = 0;
        int high_V = 0;
    }color_range;
    void ImageCallback(const sensor_msgs::msg::Image &msg);
    void FindSentry(cv::Mat &hsv_image);
    void FindEnemy(cv::Mat &hsv_image);
    void FindItems(cv::Mat &hsv_image, color_range &range, geometry_msgs::msg::Pose2D::SharedPtr &point,
        rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr point_pub);

private:
    bool is_firstcycle_; //是否第一次操作
    cv::Mat received_image_; //接收到的图片

    geometry_msgs::msg::Pose2D::SharedPtr sentry_point_;
    geometry_msgs::msg::Pose2D::SharedPtr base_point_;
    geometry_msgs::msg::Pose2D::SharedPtr center_point_;
    geometry_msgs::msg::Pose2D::SharedPtr supply_point_;
    geometry_msgs::msg::Pose2D::SharedPtr purple_point_;
    geometry_msgs::msg::Pose2D::SharedPtr green_point_;

    color_range red_range_{
        0,
        43,
        46,
        10,
        255,
        255
    };
    color_range sentry_range_{
        100,
        130,
        200,
        110,
        255,
        255
    };
    color_range center_range_{
        110,
        100,
        200,
        124,
        255,
        255
    };
    color_range supply_range_{
        100,
        43,
        46,
        110,
        220,
        145
    };
    color_range purple_range_{
        125,
        43,
        46,
        155,
        255,
        255
    };
    color_range green_range_{
        35,
        150,
        46,
        77,
        255,
        255
    };
    color_range base_range_{
        150,
        50,
        110,
        180,
        150,
        175
    };

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_; //接收图片信息
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr base_point_pub_; //发布基地坐标
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr center_point_pub_; //发布中央坐标
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr supply_point_pub_; //发布补给区坐标
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr purple_point_pub_; //发布紫色传送点坐标
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr green_point_pub_; //发布绿色传送点坐标
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr sentry_point_pub_; //发布哨兵坐标
};