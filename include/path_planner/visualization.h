#ifndef VISUALIZATION_H
#define VISUALIZATION_H

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>
#include <string>
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/Point.h"
#include "common.h"


// 可视化工具类
class Visualization
{
public:
    // 构造函数
    Visualization(ros::NodeHandle &nh, const std::string &topic);

    void setGraphData(const std::vector<Node3D*> &nodes, const std::vector<Edge3D> &edges);
    void setPathData(const std::vector<Node3D*> &path_points);


    void visualizeStartAndGoal(double start_x, double start_y, double start_z,
                                           double goal_x, double goal_y, double goal_z);
private:
    // 定时器回调函数，用于周期性发布 Marker
    void timerCallback(const ros::TimerEvent &event);



    ros::Publisher pub_start_goal;
    ros::Publisher marker_array_pub_; // MarkerArray 发布器
    ros::Timer timer_;                // 定时器

    std::vector<Node3D*> nodes_;         // 缓存的节点数据
    std::vector<Edge3D> edges_;         // 缓存的边数据
    std::vector<Node3D*> path_;         // 缓存的节点数据

    double timer_interval_;           // 定时器间隔（发布频率）
};

#endif // VISUALIZATION_H
