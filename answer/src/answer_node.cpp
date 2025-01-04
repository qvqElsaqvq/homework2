#include "answer/answer.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    std::shared_ptr<Answer> node = std::make_shared<Answer>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
