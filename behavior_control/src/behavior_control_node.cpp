#include "behavior_control/behavior_control.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    std::shared_ptr<BehaviorControl> node = std::make_shared<BehaviorControl>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
