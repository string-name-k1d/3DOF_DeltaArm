#include <memory>

#include "arm/arm_endpoint_node.hpp"

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<DeltaArmRos::ArmEndpointNode>(options);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
