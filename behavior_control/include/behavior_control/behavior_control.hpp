#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "example_interfaces/msg/int64.hpp"
#include "example_interfaces/msg/bool.hpp"
#include "opencv2/opencv.hpp"
#include <stack>
#include "cv_bridge/cv_bridge.h"
#include <iostream>
#include <memory>

#define IMAGE_HEIGHT 64
#define IMAGE_WIDTH 128

class BehaviorControl : public rclcpp::Node
{
public:
    explicit BehaviorControl();

    void imageCallback(const sensor_msgs::msg::Image &msg);
    //接收图像处理传过来的坐标点
    void basePointCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg);
    void centerPointCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg);
    void supplyPointCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg);
    void purplePointCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg);
    void greenPointCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg);
    void sentryPointCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg);
    //建图
    void BuildMap(cv::Mat &hsv_image);
    //发布
    void PublishPose();
    void PublishPassword();
    void PasswordCallback(const example_interfaces::msg::Int64::SharedPtr msg);
    void PublishShoot();

    /*
     * @brief Dijkstra，把图像中每个像素点看作一个点，相邻白点之间距离为1
     * @param：src_index：源点的索引;
     * @param：des_index：终点的索引;
     * @param：pre[]：前驱数组,即pre[i]为从src_index到i最短路径时,i前面那个顶点的索引
     * @param：dist[]：距离数组，即dist[i]是vs到i的最短路径的长度
     */
    void FindPath(int src_index, int des_index);
    void GetMartix(); //获得邻接矩阵
    void MoveControl();
    void GetPath(); //从终点的索引倒推至源点的索引
    // void timerCallback();

private:
    int password_; //密码
    int password_cnt_; //密码计数
    double pose_x_; //发布的
    double pose_y_; //发布的
    double pose_theta_; //发布的（绝对角度，弧度制）
    bool is_shoot_; //是否射击
    cv::Mat map_image_;
    geometry_msgs::msg::Pose2D::SharedPtr base_point_;
    geometry_msgs::msg::Pose2D::SharedPtr center_point_;
    geometry_msgs::msg::Pose2D::SharedPtr supply_point_;
    geometry_msgs::msg::Pose2D::SharedPtr purple_point_;
    geometry_msgs::msg::Pose2D::SharedPtr green_point_;
    geometry_msgs::msg::Pose2D::SharedPtr sentry_point_;

    double target_x_; //目标在处理过图像中的坐标
    double target_y_;
    double target_theta_; //目标角度
    int current_index_; //当前索引
    int target_index_; //目标点索引
    int current_step_;
    int target_step_; //决定下一个目标点是哪里
    std::stack<int> path_; //索引路径

    /*坐标点索引，原因是图像的邻接矩阵为IMAGE_HEIGHT*IMAGE_WIDTH x IMAGE_HEIGHT*IMAGE_WIDTH，而平时查图像是IMAGE_HEIGHT x IMAGE_WIDTH
     *因此多了一步这个转换，怀疑在来回转换的过程中有问题？
     */
    int base_index_;
    int center_index_;
    int supply_index_;
    int purple_index_;
    int green_index_;
    int sentry_index_;

    bool is_first_time_;
    // int step_length_;
    bool is_reached_; //是否到达目标点

    enum Target{BASE = 0, CENTER = 1, SUPPLY = 2, PURPLE = 3, GREEN = 4};

    int prev[IMAGE_HEIGHT * IMAGE_WIDTH] = {0};
    int dist[IMAGE_HEIGHT * IMAGE_WIDTH] = {0};
    int martix[IMAGE_HEIGHT*IMAGE_WIDTH][IMAGE_HEIGHT*IMAGE_WIDTH];

    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr pose_pub_; //发布位姿
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr password_sub_; //接收密码片段
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr password_pub_; //发布密码
    rclcpp::Publisher<example_interfaces::msg::Bool>::SharedPtr shoot_pub_; //发布是否射击
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr map_sub_; //接收地图信息
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr base_point_sub_; //基地坐标
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr center_point_sub_; //中央坐标
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr supply_point_sub_; //补给区坐标
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr purple_point_sub_; //紫色传送点坐标
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr green_point_sub_; //绿色传送点坐标
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr sentry_point_sub_; //哨兵坐标
    // rclcpp::TimerBase::SharedPtr timer_;
};