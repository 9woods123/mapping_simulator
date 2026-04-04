#include "path_planner/rrtstar.h"
#include <chrono>

RRTStarPlanner::RRTStarPlanner(std::shared_ptr<mapping_simulator::MappingSimulator> ms,
                               bool is_2D, double fixed_z)
    : ms_(ms), is_2D_(is_2D), fixed_z_(fixed_z)
{
    if (!ms_) throw std::runtime_error("MappingSimulator is null");

    ms_->getMapBounds(min_x_, min_y_, min_z_, max_x_, max_y_, max_z_);

    rng_ = std::mt19937(std::random_device{}());
    dist_x_ = std::uniform_real_distribution<double>(min_x_, max_x_);
    dist_y_ = std::uniform_real_distribution<double>(min_y_, max_y_);
    dist_z_ = std::uniform_real_distribution<double>(min_z_, max_z_);
}

void RRTStarPlanner::setStart(double x, double y, double z)
{
    start_ = Eigen::Vector3d(x, y, is_2D_ ? fixed_z_ : z);
}

void RRTStarPlanner::setGoal(double x, double y, double z)
{
    goal_ = Eigen::Vector3d(x, y, is_2D_ ? fixed_z_ : z);
}

Eigen::Vector3d RRTStarPlanner::sampleRandom()
{
    return Eigen::Vector3d(dist_x_(rng_), dist_y_(rng_), is_2D_ ? fixed_z_ : dist_z_(rng_));
}

double RRTStarPlanner::distance(const Eigen::Vector3d& a, const Eigen::Vector3d& b) const
{
    return (a - b).norm();
}

Node3D* RRTStarPlanner::nearestNeighbor(const Eigen::Vector3d& pt)
{
    Node3D* nearest = nullptr;
    double min_dist = std::numeric_limits<double>::max();
    for (auto node : nodes_)
    {
        double d = distance(pt, node->pos);
        if (d < min_dist)
        {
            min_dist = d;
            nearest = node;
        }
    }
    return nearest;
}

// 离散步长碰撞检测
bool RRTStarPlanner::isSegmentFree(const Eigen::Vector3d& p1, const Eigen::Vector3d& p2, double step)
{
    Eigen::Vector3d dir = p2 - p1;
    double len = dir.norm();
    dir.normalize();

    int N = std::ceil(len / step);
    for (int i = 0; i <= N; ++i)
    {
        Eigen::Vector3d pt = p1 + i * step * dir;
        if (ms_->isPointOccupiedWithVolume(pt.x(), pt.y(), pt.z(), 0.1))
            return false;
    }
    return true;
}

// 核心 RRT* 算法
bool RRTStarPlanner::solve(double time_limit, double step_size, double goal_radius)
{

    nodes_.clear();
    edges_.clear();

    auto root = new Node3D{start_, nullptr, 0.0};  // pos, parent, cost

    nodes_.push_back(root);

    auto t_start = std::chrono::steady_clock::now();

    while (true)
    {
        auto t_now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(t_now - t_start).count();
        
        if (elapsed > time_limit)
            break;

        Eigen::Vector3d rnd = sampleRandom();

        Node3D* nearest = nearestNeighbor(rnd);
        Eigen::Vector3d dir = rnd - nearest->pos;
        double len = dir.norm();
        if (len > step_size) dir = dir / len * step_size;
        Eigen::Vector3d new_pos = nearest->pos + dir;

        if (!isSegmentFree(nearest->pos, new_pos)) continue;

        // 添加节点
        Node3D* new_node = new Node3D{new_pos, nearest, nearest->cost + distance(nearest->pos, new_pos)};
        nodes_.push_back(new_node);
        edges_.push_back({nearest, new_node});

        // RRT* rewire
        rewire(new_node, step_size * 5);

        // 检查是否到达 goal
        if (distance(new_node->pos, goal_) < goal_radius && isSegmentFree(new_node->pos, goal_))
        {
            Node3D* goal_node = new Node3D{goal_, new_node, new_node->cost + distance(new_node->pos, goal_)};
            nodes_.push_back(goal_node);
            edges_.push_back({new_node, goal_node});
            return true;
        }
    }

    return false;
}

// RRT* 重新连接附近节点
void RRTStarPlanner::rewire(Node3D* new_node, double radius)
{
    for (auto node : nodes_)
    {
        if (node == new_node) continue;
        if (distance(new_node->pos, node->pos) < radius)
        {
            if (isSegmentFree(new_node->pos, node->pos))
            {
                double new_cost = new_node->cost + distance(new_node->pos, node->pos);
                if (new_cost < node->cost)
                {
                    node->parent = new_node;
                    node->cost = new_cost;
                    edges_.push_back({new_node, node});
                }
            }
        }
    }
}

// 提取最终路径
std::vector<Node3D*> RRTStarPlanner::getSolutionPath() const
{
    std::vector<Node3D*> path;
    // 找离 goal 最近的节点
    Node3D* best = nullptr;
    double min_dist = std::numeric_limits<double>::max();
    for (auto node : nodes_)
    {
        double d = distance(node->pos, goal_);
        if (d < min_dist)
        {
            min_dist = d;
            best = node;
        }
    }

    while (best)
    {
        path.push_back(best);
        best = best->parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}

// 提取完整图
std::pair<std::vector<Node3D*>, std::vector<Edge3D>> RRTStarPlanner::getGraph() const
{
    return {nodes_, edges_};
}


void RRTStarPlanner::sampleValidStartGoal(
    double& sx, double& sy, double& sz,
    double& gx, double& gy, double& gz, double distance)
{
    int max_trials = 1000;   // 防止死循环

    for (int i = 0; i < max_trials; ++i)
    {
        // ===== 随机采样 =====
        sx = dist_x_(rng_);
        sy = dist_y_(rng_);

        // 地上 目标
        std::uniform_real_distribution<double> onground_dist_z = std::uniform_real_distribution<double>(0.5*max_z_, max_z_);

        sz = is_2D_ ? fixed_z_ : onground_dist_z(rng_);

        gx = dist_x_(rng_);
        gy = dist_y_(rng_);
        gz = is_2D_ ? fixed_z_ : onground_dist_z(rng_);

        // ===== 碰撞检测 =====
        bool start_valid = !ms_->isPointOccupiedWithVolume(sx, sy, sz, 0.3);
        bool goal_valid  = !ms_->isPointOccupiedWithVolume(gx, gy, gz, 0.3);

        // ===== 距离约束（非常重要 ⭐）=====
        double dist = (Eigen::Vector3d(sx, sy, sz) -
                       Eigen::Vector3d(gx, gy, gz)).norm();

        if (start_valid && goal_valid && dist > distance)
        {
            break;
        }
    }


}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "rrtstar_test");
    ros::NodeHandle nh;

    Visualization vis(nh, "rrtstar_vis");

    // ===== 地图 =====
    auto ms = std::make_shared<mapping_simulator::MappingSimulator>();

    // 如果你有 map 文件，记得打开
    // ms->resetMap("xxx.bt");

    // ===== RRT* =====
    RRTStarPlanner planner(ms, false, 1.0);

    // ===== start / goal =====
    double sx = -10, sy = -10, sz = 1.5;
    double gx =  10, gy = 10, gz = 2.0;

    planner.setStart(sx, sy, sz);
    planner.setGoal(gx, gy, gz);

    ROS_INFO("Start planning...");

    bool success = planner.solve(
        8.0,   // time limit
        1.0,   // step size ⭐ 很关键
        1.0    // goal radius
    );

    if (success)
    {
        ROS_INFO("RRT* success!");

        // ===== path =====
        auto path_nodes = planner.getSolutionPath();

        vis.setPathData(path_nodes);
    }
    else
    {
        ROS_WARN("RRT* failed");
    }


    // ===== graph =====
    auto [nodes, edges] = planner.getGraph();

    vis.setGraphData(nodes, edges);

    vis.visualizeStartAndGoal(sx, sy, sz, gx, gy, gz);

    ros::spin();
    return 0;
}