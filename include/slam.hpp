#pragma once
#include <vector>
#include <string>
#include <Eigen/Dense>

namespace PoseGraph {
class Pose {
public:
    Eigen::Vector2d position; // (x,y)
    double orientation; // theta
    
    Pose() = default;
    Pose(double x, double y, double theta);

    Eigen::Matrix3d toTransformationMatrix() const; // pose -> homogeneous transformation matrix
};

static Pose fromTransformationMatrix(const Eigen::Matrix3d& matrix); // homogeneous transformation matrix -> pose

class Edge {
public:
    u_int64_t from; // First Pose index
    u_int64_t to; // Second Pose index
    Pose measurement;
    Eigen::Matrix3d information;

    Edge(int from, int to, const Pose& measurement, const Eigen::Matrix3d& information);
};

class PoseGraph {
public:
    PoseGraph() = default;
    std::vector<Pose> poses;
    std::vector<Edge> edges;

    void addPose(const Pose& pose, u_int32_t index);
    void addEdge(const Edge& edge);
    void optimise(int iteration);
    void importEdges(std::string filename);
    void importPoses(std::string filename);

private:
    void computeErrorAndJacobians(Eigen::MatrixXd& H, Eigen::VectorXd& b);
};
} // namespace PoseGraph