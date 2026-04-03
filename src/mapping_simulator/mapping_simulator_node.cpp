#include <ros/ros.h>
#include "mapping_simulator/mapping_simulator.h"

int main(int argc, char** argv) {
    // 初始化 ROS 节点
    ros::init(argc, argv, "mapping_simulator_node");
    ros::NodeHandle nh;

    // 创建 ESDFMapGenerator 对象
    mapping_simulator::MappingSimulator mapping_simulator;

    // 启动 ROS 循环
    ros::spin();

    return 0;
}
