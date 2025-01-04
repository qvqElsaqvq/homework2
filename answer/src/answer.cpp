#include "answer/answer.hpp"

Answer::Answer() : Node("answer_node")
{
    is_firstcycle_ = true;
    sentry_point_ = std::make_shared<geometry_msgs::msg::Pose2D>();
    base_point_ = std::make_shared<geometry_msgs::msg::Pose2D>();
    center_point_ = std::make_shared<geometry_msgs::msg::Pose2D>();
    supply_point_ = std::make_shared<geometry_msgs::msg::Pose2D>();
    purple_point_ = std::make_shared<geometry_msgs::msg::Pose2D>();
    green_point_ = std::make_shared<geometry_msgs::msg::Pose2D>();

    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>("image_raw", 10,
        std::bind(&Answer::ImageCallback, this, std::placeholders::_1));
    base_point_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("base_point", 10);
    center_point_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("center_point", 10);
    supply_point_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("supply_point", 10);
    purple_point_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("purple_point", 10);
    green_point_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("green_point", 10);
    sentry_point_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("sentry_point", 10);
}

void Answer::ImageCallback(const sensor_msgs::msg::Image &msg) {
    cv_bridge::CvImagePtr cvImage;
    cvImage = cv_bridge::toCvCopy( msg, msg.encoding);
    cvImage->image.copyTo(received_image_); //获取ROS传来的图片
    cv::cvtColor(received_image_,received_image_,cv::COLOR_RGB2BGR);
    cv::Mat hsv_image;
    cv::cvtColor(received_image_, hsv_image, cv::COLOR_BGR2HSV);
    if(is_firstcycle_) { //第一次操作确定场地道具坐标
        std::cout << received_image_.cols << " " << received_image_.rows << std::endl;
        is_firstcycle_ = false;
        FindItems(hsv_image, base_range_, base_point_, base_point_pub_);
        FindItems(hsv_image, center_range_, center_point_, center_point_pub_);
        FindItems(hsv_image, supply_range_, supply_point_, supply_point_pub_);
        FindItems(hsv_image, purple_range_, purple_point_, purple_point_pub_);
        FindItems(hsv_image, green_range_, green_point_, green_point_pub_);
    }
    FindSentry(hsv_image);
    FindEnemy(hsv_image);
}

void Answer::FindEnemy(cv::Mat &hsv_image) {
    //颜色提取
    cv::Mat threshold_image; //颜色提取后的图片
    std::vector<std::vector<cv::Point> > contours; //每一组Point点集就是一个轮廓
    std::vector<cv::Vec4i> hierarchy;
    cv::inRange(hsv_image, cv::Scalar(red_range_.low_H, red_range_.low_S, red_range_.low_V),
        cv::Scalar(red_range_.high_H, red_range_.high_S, red_range_.high_V), threshold_image);
    cv::Mat element_open, element_close; //开闭操作
    element_open = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    element_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(13, 13));
    cv::morphologyEx(threshold_image, threshold_image, cv::MORPH_OPEN, element_open); //开操作
    cv::morphologyEx(threshold_image, threshold_image, cv::MORPH_CLOSE, element_close); //闭操作
    // cv::imshow("Enemy", threshold_image);
    // cv::waitKey();

    //提取轮廓
    cv::Mat blurred_image; //滤波和边缘检测后的图片
    cv::Canny(threshold_image, blurred_image, 0.0, 255.0); //边缘检测
    cv::findContours(blurred_image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    //计算敌人中心坐标并存储
    std::vector<cv::Point2f> enemy_points;
    for(auto &contour : contours) {
		//对给定的2D点集，寻找最小面积的包围矩形
		cv::RotatedRect box = cv::minAreaRect(cv::Mat(contour));
		cv::Point2f vertex[4];
		box.points(vertex);
		//中心坐标
		cv::Point2f point;
		point.x = (vertex[0].x + vertex[2].x) / 2.0;
		point.y = (vertex[0].y + vertex[2].y) / 2.0;
        enemy_points.push_back(point); //寻找敌人中心坐标
    }
}

void Answer::FindSentry(cv::Mat &hsv_image) {
    //颜色提取
    cv::Mat threshold_image; //颜色提取后的图片
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::inRange(hsv_image, cv::Scalar(sentry_range_.low_H, sentry_range_.low_S, sentry_range_.low_V),
        cv::Scalar(sentry_range_.high_H, sentry_range_.high_S, sentry_range_.high_V), threshold_image);
    cv::Mat element_open, element_close; //开闭操作
    element_open = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    element_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(13, 13));
    cv::morphologyEx(threshold_image, threshold_image, cv::MORPH_OPEN, element_open); //开操作
    cv::morphologyEx(threshold_image, threshold_image, cv::MORPH_CLOSE, element_close); //闭操作
    // cv::imshow("Sentry", threshold_image);
    // cv::waitKey();

    //提取轮廓
    cv::Mat blurred_image; //滤波和边缘检测后的图片
    cv::Canny(threshold_image, blurred_image, 0.0, 255.0); //边缘检测
    cv::findContours(blurred_image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    //中心坐标，即哨兵坐标
    cv::Point2f point;
    for(auto &contour : contours) {
        //对给定的2D点集，寻找最小面积的包围矩形
        cv::RotatedRect box = cv::minAreaRect(cv::Mat(contour));
        cv::Point2f vertex[4];
        box.points(vertex);

        point.x = (vertex[0].x + vertex[2].x) / 2.0;
        point.y = (vertex[0].y + vertex[2].y) / 2.0;
        sentry_point_->x = point.x;
        sentry_point_->y = point.y;
        sentry_point_->theta = 0.0;
        sentry_point_pub_->publish(*sentry_point_);
        // cv::rectangle(blurred_image, vertex[0], vertex[2], cv::Scalar(255,255,255),6);
        // cv::imshow("Sentry", blurred_image);
        // cv::waitKey(1);
    }
}

void Answer::FindItems(cv::Mat &hsv_image, color_range &range, geometry_msgs::msg::Pose2D::SharedPtr &point,
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr point_pub) {
    //颜色提取
    cv::Mat threshold_image; //颜色提取后的图片
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::inRange(hsv_image, cv::Scalar(range.low_H, range.low_S, range.low_V),
        cv::Scalar(range.high_H, range.high_S, range.high_V), threshold_image);
    cv::Mat element_open, element_close; //开闭操作
    element_open = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    element_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(13, 13));
    cv::morphologyEx(threshold_image, threshold_image, cv::MORPH_OPEN, element_open); //开操作
    cv::morphologyEx(threshold_image, threshold_image, cv::MORPH_CLOSE, element_close); //闭操作

    //提取轮廓
    cv::Mat blurred_image; //滤波和边缘检测后的图片
    cv::Canny(threshold_image, blurred_image, 0.0, 255.0); //边缘检测
    cv::findContours(blurred_image, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    //中心坐标
    cv::Point2f cv_point;
    for(auto &contour : contours) {
        //对给定的2D点集，寻找最小面积的包围矩形
        cv::RotatedRect box = cv::minAreaRect(cv::Mat(contour));
        cv::Point2f vertex[4];
        box.points(vertex);

        cv_point.x = (vertex[0].x + vertex[2].x) / 2.0;
        cv_point.y = (vertex[0].y + vertex[2].y) / 2.0;
        point->x = cv_point.x;
        point->y = cv_point.y;
        point->theta = 0.0;
        point_pub->publish(*point);
        cv::rectangle(blurred_image, vertex[0], vertex[2], cv::Scalar(255,255,255),6);
        // cv::imshow("Items", blurred_image);
        // cv::waitKey(0);
    }
}
