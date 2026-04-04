#ifndef RRT_STAR_PLANNER_H
#define RRT_STAR_PLANNER_H

#include <iostream>
#include <vector>
#include <memory>
#include <random>
#include <cmath>
#include <limits>
#include <queue>
#include <Eigen/Dense>
#include "mapping_simulator/mapping_simulator.h"
#include "visualization.h"
#include "common.h"


class RRTStarPlanner
{
public:
    RRTStarPlanner(std::shared_ptr<mapping_simulator::MappingSimulator> ms,
                   bool is_2D = false, double fixed_z = 0.5);

    void setStart(double x, double y, double z);
    void setGoal(double x, double y, double z);

    bool solve(double time_limit, double step_size = 0.5, double goal_radius = 0.5);

    std::vector<Node3D*> getSolutionPath() const;
    std::pair<std::vector<Node3D*>, std::vector<Edge3D>> getGraph() const;

private:
    std::shared_ptr<mapping_simulator::MappingSimulator> ms_;
    bool is_2D_;
    double fixed_z_;

    Eigen::Vector3d start_;
    Eigen::Vector3d goal_;

    std::vector<Node3D*> nodes_;
    std::vector<Edge3D> edges_;

    // 地图边界
    double min_x_, min_y_, min_z_;
    double max_x_, max_y_, max_z_;

    // 随机数生成
    std::mt19937 rng_;
    std::uniform_real_distribution<double> dist_x_;
    std::uniform_real_distribution<double> dist_y_;
    std::uniform_real_distribution<double> dist_z_;

    Eigen::Vector3d sampleRandom();
    Node3D* nearestNeighbor(const Eigen::Vector3d& pt);
    bool isSegmentFree(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double step = 0.1);
    double distance(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const;
    void rewire(Node3D* new_node, double radius = 1.0);
    
};

#endif // RRT_STAR_PLANNER_H