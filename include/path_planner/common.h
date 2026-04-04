#ifndef COMMON_H
#define COMMON_H


#include <vector>
#include <string>
#include <Eigen/Dense>

struct Node3D
{
    Eigen::Vector3d pos;
    Node3D* parent = nullptr;
    double cost = 0.0; // 累积代价
};

struct Edge3D
{
    Node3D* from;
    Node3D* to;
};

#endif // COMMON_H