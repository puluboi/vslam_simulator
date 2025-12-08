#include <iostream>
#include "orb.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    Orb_Basic orb_basic(2000);
    rclcpp::shutdown();
    return 0;
}
