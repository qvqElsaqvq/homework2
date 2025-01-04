#include "behavior_control/behavior_control.hpp"

#define INF 5000

BehaviorControl::BehaviorControl() : Node("behavior_control_node")
{
    password_ = 0;
    password_cnt_ = 0;
    pose_theta_ = 0.0;
    pose_x_ = 0.0;
    pose_y_ = 0.0;
    is_shoot_ = false;
    target_index_ = 0;

    base_point_ = std::make_shared<geometry_msgs::msg::Pose2D>();
    center_point_ = std::make_shared<geometry_msgs::msg::Pose2D>();
    supply_point_ = std::make_shared<geometry_msgs::msg::Pose2D>();
    purple_point_ = std::make_shared<geometry_msgs::msg::Pose2D>();
    green_point_ = std::make_shared<geometry_msgs::msg::Pose2D>();
    sentry_point_ = std::make_shared<geometry_msgs::msg::Pose2D>();

    target_x_ = 0.0;
    target_y_ = 0.0;
    target_theta_ = 0.0;
    current_step_ = 0;
    is_first_time_ = true;
    target_step_ = SUPPLY;

    sentry_point_->x = 0.0;
    sentry_point_->y = 0.0;

    //邻接矩阵初始化
    for(int i = 0; i < IMAGE_HEIGHT*IMAGE_WIDTH; i++) {
        for(int j = 0; j < IMAGE_HEIGHT*IMAGE_WIDTH; j++) {
            martix[i][j] = INF;
        }
    }

    map_sub_ = this->create_subscription<sensor_msgs::msg::Image>("image_raw", 10,
        std::bind(&BehaviorControl::imageCallback, this, std::placeholders::_1));
    base_point_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>("base_point", 10,
        std::bind(&BehaviorControl::basePointCallback, this, std::placeholders::_1));
    center_point_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>("center_point", 10,
        std::bind(&BehaviorControl::centerPointCallback, this, std::placeholders::_1));
    supply_point_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>("supply_point", 10,
        std::bind(&BehaviorControl::supplyPointCallback, this, std::placeholders::_1));
    purple_point_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>("purple_point", 10,
        std::bind(&BehaviorControl::purplePointCallback, this, std::placeholders::_1));
    green_point_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>("green_point", 10,
        std::bind(&BehaviorControl::greenPointCallback, this, std::placeholders::_1));
    sentry_point_sub_ = this->create_subscription<geometry_msgs::msg::Pose2D>("sentry_point", 10,
        std::bind(&BehaviorControl::sentryPointCallback, this, std::placeholders::_1));
    pose_pub_ = this->create_publisher<geometry_msgs::msg::Pose2D>("pose", 10);
    password_pub_ = this->create_publisher<example_interfaces::msg::Int64>("password", 10);
    password_sub_ = this->create_subscription<example_interfaces::msg::Int64>("password_segment", 10,
        std::bind(&BehaviorControl::PasswordCallback, this, std::placeholders::_1));
    shoot_pub_ = this->create_publisher<example_interfaces::msg::Bool>("shoot", 10);
    // auto period_ms = std::chrono::milliseconds(static_cast<int64_t>(10)); // 10ms
    // timer_ = rclcpp::create_timer(this, this->get_clock(), period_ms,
    //                               std::bind(&BehaviorControl::timerCallback, this));
}

void BehaviorControl::PublishPassword() {
    example_interfaces::msg::Int64::SharedPtr msg;
    msg->data = password_;
    password_pub_->publish(*msg);
}

void BehaviorControl::PublishPose() {
    geometry_msgs::msg::Pose2D::SharedPtr msg;
    msg->theta = pose_theta_;
    msg->x = pose_x_;
    msg->y = pose_y_;
    pose_pub_->publish(*msg);
}

void BehaviorControl::PublishShoot() {
    example_interfaces::msg::Bool::SharedPtr msg;
    msg->data = is_shoot_;
    shoot_pub_->publish(*msg);
}

void BehaviorControl::PasswordCallback(const example_interfaces::msg::Int64::SharedPtr msg) {
    password_ += msg->data;
}

void BehaviorControl::BuildMap(cv::Mat &hsv_image) {
    //颜色提取
    cv::Mat threshold_image; //颜色提取后的图片
    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::inRange(hsv_image, cv::Scalar(0, 0, 46), cv::Scalar(180, 25, 100), threshold_image);
    cv::resize(threshold_image, map_image_,
        cv::Size(threshold_image.cols / 16, threshold_image.rows / 16));

    //取反
    int height = map_image_.rows;
    int width = map_image_.cols;
    int channel = map_image_.channels();
    //数组方法遍历
    for (int h = 0; h < height; h++)
    {
        for (int w = 0; w < width; w++)
        {
            if (channel == 1)//灰度图像
            {
                int value = map_image_.at<uchar>(h, w);
                map_image_.at<uchar>(h, w) = 255 - value;
            }
        }
    }
    // cv::imshow("map", map_image_);
    // cv::waitKey(0);
    GetMartix();
    current_step_ = 1;
}

void BehaviorControl::imageCallback(const sensor_msgs::msg::Image &msg) {
    if(is_first_time_) {
        cv::Mat received_image;
        cv_bridge::CvImagePtr cvImage;
        cvImage = cv_bridge::toCvCopy( msg, msg.encoding);
        cvImage->image.copyTo(received_image); //获取ROS传来的图片
        cv::cvtColor(received_image,received_image,cv::COLOR_RGB2BGR);
        cv::Mat hsv_image;
        cv::cvtColor(received_image, hsv_image, cv::COLOR_BGR2HSV);
        BuildMap(hsv_image);
        is_first_time_ = false;
    }
}

void BehaviorControl::basePointCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
    base_point_->x = msg->x;
    base_point_->y = msg->y;
    base_point_->theta = msg->theta;
    base_index_ = int((base_point_->y - 1) / 16 * IMAGE_WIDTH) + int(base_point_->x / 16);
}

void BehaviorControl::centerPointCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
    center_point_->x = msg->x;
    center_point_->y = msg->y;
    center_point_->theta = msg->theta;
    center_index_ = int((center_point_->y - 1) / 16 * IMAGE_WIDTH) + int(center_point_->x / 16);
}

void BehaviorControl::greenPointCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
    green_point_->x = msg->x;
    green_point_->y = msg->y;
    green_point_->theta = msg->theta;
    green_index_ = int((green_point_->y - 1) / 16 * IMAGE_WIDTH) + int(green_point_->x / 16);
}

void BehaviorControl::purplePointCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
    purple_point_->x = msg->x;
    purple_point_->y = msg->y;
    purple_point_->theta = msg->theta;
    purple_index_ = int((purple_point_->y - 1) / 16 * IMAGE_WIDTH) + int(purple_point_->x / 16);
}

void BehaviorControl::supplyPointCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
    supply_point_->x = msg->x;
    supply_point_->y = msg->y;
    supply_point_->theta = msg->theta;
    supply_index_ = int((supply_point_->y - 1) / 16 * IMAGE_WIDTH) + int(supply_point_->x / 16);
}

//从这个回调控制决策，预想的情况是哨兵每走一步都能更新要发布的pose
void BehaviorControl::sentryPointCallback(const geometry_msgs::msg::Pose2D::SharedPtr msg) {
    sentry_point_->x = msg->x;
    sentry_point_->y = msg->y;
    sentry_point_->theta = msg->theta;
    //从原图到map有缩小
    sentry_index_ = int((sentry_point_->y - 1) / 16 * IMAGE_WIDTH) + int(sentry_point_->x / 16);
    // std::cout << sentry_point_->x << " " << sentry_point_->y << std::endl;

    RCLCPP_INFO(this->get_logger(), "Current step is %d", current_step_);
    if(current_step_ == 0) { //建图
        RCLCPP_INFO(this->get_logger(), "Waiting map!");
    }
    else if(current_step_ == 1) { //计算路径
        RCLCPP_INFO(this->get_logger(), "Waiting path!");
        if(sentry_point_->x != 0) {
            FindPath(sentry_index_, supply_index_);
            if(target_step_ == SUPPLY) { //现在只写了SUPPLY一个
                target_index_ = supply_index_;
                std::cout << "-----SUPPLY-----" << std::endl;
            }
            GetPath();
        }
    }
    else if(current_step_ == 2) { //控制导航步骤
        if(!path_.empty()) {
            current_index_ = path_.top();
            std::cout << "Current step is %d" << current_index_ << std::endl;
            MoveControl();
            if(sentry_index_ == current_index_)
                path_.pop();
        }
        else {
            std::cout << "navigation finished!" << std::endl;
        }
    }
}

void BehaviorControl::FindPath(int src_index, int des_index) {
    int i,j,k;
    int min;
    int tmp;
    int flag[IMAGE_HEIGHT * IMAGE_WIDTH];  // flag[i]=1表示"顶点vs"到"顶点i"的最短路径已成功获取。
    current_index_ = src_index;

    //初始化参数
    for (i = 0; i < IMAGE_HEIGHT * IMAGE_WIDTH; i++)
    {
        flag[i] = 0;  // 顶点i的最短路径还没获取到。
        prev[i] = src_index;  // 顶点i的前驱顶点为src_index。
        dist[i] = martix[src_index][i]; // 顶点i的最短路径为vs到i的权。
    }
    flag[src_index] = 1; // 对顶点vs自身进行初始化
    dist[src_index] = 0;

    //遍历q-1次，每次找出vs到另一个顶点的最短路径
    for (i = 1; i < IMAGE_HEIGHT * IMAGE_WIDTH; i++)
    {
        k = -1;
        /* 在未获取最短路径的顶点中，找到离vs最近的顶点k */
        min = INF;
        for ( j = 0; j < IMAGE_HEIGHT * IMAGE_WIDTH; j++)
        {
            if (flag[j] == 0 && dist[j] < min)
			//若从vs到顶点j距离小于min,而且从vs到j的最短路径还未获取。 
            {
                min = dist[j];//改变最近距离
                k = j;//记录j 
            }
        }
        
       /* 对刚刚已找到最短距离的顶点k进行标记判断 */
        if(k != -1) {
            flag[k] = 1; // 标记顶点k,dist[k]已确定。
            if(k == des_index)   //判断k是否是终点索引,若是则退出
			    break;

            /* 已知顶点k的最短路径后,更新未获取最短路径的顶点的最短路径和前驱顶点 */
            for (j = 0; j < IMAGE_HEIGHT * IMAGE_WIDTH; j++)
            {
                tmp = (martix[k][j%IMAGE_WIDTH]==INF ? INF : (min + martix[k][j%IMAGE_WIDTH])); // 防止溢出
                if (flag[j] == 0 && tmp  < dist[j]) //若j还不是最短距离且从k到j距离比记录的距离短
                {
                    //更新k的前驱和最短距离
			        prev[j] = k;
                    dist[j] = tmp;
                }
            }
        }
    }
    current_step_ = 2;
}

void BehaviorControl::GetMartix() {
    int i, j;
    for (i = 0; i < IMAGE_HEIGHT; i++) {
        for(j = 0; j < IMAGE_WIDTH; j++) {
            int value = map_image_.at<uchar>(i, j);
            if (value != 0) {
                //当前格子不是黑色
                if (i != 0 && map_image_.at<uchar>(i - 1, j) != 0) {
                    martix[int(i * IMAGE_WIDTH + j)][int((i - 1) * IMAGE_WIDTH + j)] = 1;
                }
                if (j != 0 && map_image_.at<uchar>(i, j - 1) != 0) {
                    martix[int(i * IMAGE_WIDTH + j)][int(i * IMAGE_WIDTH + j - 1)] = 1;
                }
                if (i != IMAGE_HEIGHT - 1 && map_image_.at<uchar>(i + 1, j) != 0) {
                    martix[int(i * IMAGE_WIDTH + j)][int((i + 1) * IMAGE_WIDTH + j)] = 1;
                }
                if (j != IMAGE_WIDTH - 1 && map_image_.at<uchar>(i, j + 1) != 0) {
                    martix[int(i * IMAGE_WIDTH + j)][int(i * IMAGE_WIDTH + j + 1)] = 1;
                }
            }
        }
    }
}

void BehaviorControl::GetPath() {
    int index = target_index_;
    while(index != sentry_index_) {
        path_.push(index);
        index = prev[index];
    }
}


void BehaviorControl::MoveControl() {
    //将前序点的位置转换为前序点坐标
    target_x_ = current_index_ % IMAGE_WIDTH;
    target_y_ = current_index_ / IMAGE_WIDTH;
    std::cout << "target_x: " << target_x_ << " target_y: " << target_y_ << std::endl;

    pose_x_ = target_x_ - sentry_point_->x / 16;
    pose_y_ = target_y_ - sentry_point_->y / 16;
    // if(target_x_ < sentry_point_->x) {
    //     pose_x_ = -1.0;
    // }
    // else if(target_x_ > sentry_point_->x) {
    //     pose_x_ = 1.0;
    // }
    // else {
    //     pose_x_ = 0.0;
    // }
    // if(target_y_ < sentry_point_->y) {
    //     pose_y_ = -1.0;
    // }
    // else if(target_y_ > sentry_point_->y) {
    //     pose_y_ = 1.0;
    // }
    // else {
    //     pose_y_ = 0.0;
    // }
    pose_theta_ = 0.0;
    geometry_msgs::msg::Pose2D msg;
    msg.x = pose_x_;
    msg.y = pose_y_;
    msg.theta = pose_theta_;
    // std::cout << pose_x_ << " " << pose_y_ << " " << pose_theta_ << std::endl;
    pose_pub_->publish(msg);
}

// void BehaviorControl::timerCallback() {
//
// }
