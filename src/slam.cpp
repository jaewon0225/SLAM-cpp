#include <cmath>
#include <iostream>
#include <fstream>
#include "slam.hpp"
#include "threadpool.hpp"
#define MULTITHREAD 1

namespace PoseGraph{
Pose::Pose(double x, double y, double theta)
    : position(x,y), orientation(theta) {}

Eigen::Matrix3d Pose::toTransformationMatrix() const {
    Eigen::Matrix3d transformation;
    transformation << cos(orientation), -sin(orientation), position.x(),
                      sin(orientation),  cos(orientation), position.y(),
                      0,                 0,                1;
    return transformation;
}

static Pose fromTransformationMatrix(const Eigen::Matrix3d& matrix) {
    double x = matrix(0,2);
    double y = matrix(1,2);
    double theta = atan2(matrix(1,0), matrix(0,0));
    return Pose(x, y, theta);
}

Edge::Edge(int from, int to, const Pose& measurement, const Eigen::Matrix3d& information)
    : from(from), to(to), measurement(measurement), information(information) {}

void PoseGraph::addPose(const Pose& pose, u_int32_t index) {
    poses.push_back(pose);
}

void PoseGraph::addEdge(const Edge& edge) {
    edges.push_back(edge);
}

void PoseGraph::computeErrorAndJacobians(Eigen::MatrixXd& H, Eigen::VectorXd& b) {
#if !MULTITHREAD 
    for (const Edge& edge : edges ) {
        u_int64_t i = edge.from;
        u_int64_t j = edge.to;

        const Pose& xi = poses[i];
        const Pose& xj = poses[j];
        const Pose& zij = edge.measurement;

        // Compute error term
        Eigen::Matrix3d Ti = xi.toTransformationMatrix();
        Eigen::Matrix3d Tj = xj.toTransformationMatrix();
        Eigen::Matrix3d Tij = zij.toTransformationMatrix();
        Eigen::Matrix3d eij = Tij.inverse() * Ti.inverse() * Tj;
        Pose eij_pose = fromTransformationMatrix(eij);
        Eigen::Vector3d eij_vec;
        eij_vec << eij_pose.position[0], eij_pose.position[1], eij_pose.orientation;

        Eigen::Matrix2d Ri = Ti.topLeftCorner<2, 2>();
        Eigen::Matrix2d Rij = Tij.topLeftCorner<2, 2>();

        double s = sin(xi.orientation);
        double c = cos(xi.orientation);

        Eigen::Matrix2d dRi;
        dRi << -s, -c,
               c, -s;
        
        Eigen::Vector2d dtij;
        dtij << xj.position[0] - xi.position[0], 
                xj.position[1] - xi.position[1];

        Eigen::Matrix3d Aij = Eigen::MatrixXd::Zero(3,3);
        Aij.block<2,2>(0,0) = - Rij.transpose() * Ri.transpose();
        Aij.block<2,1>(0,2) = Rij.transpose() * dRi.transpose() * dtij;
        Aij(2,2) = -1;

        Eigen::Matrix3d Bij = Eigen::MatrixXd::Zero(3,3);
        Bij.block<2,2>(0,0) = Rij.transpose() * Ri.transpose();
        Bij(2,2) = 1;

        Eigen::Matrix3d Hii = Aij.transpose() * edge.information * Aij;
        Eigen::Matrix3d Hij = Aij.transpose() * edge.information * Bij;
        Eigen::Matrix3d Hji = Hij.transpose();
        Eigen::Matrix3d Hjj = Bij.transpose() * edge.information * Bij;

        Eigen::Vector3d bi  = Aij.transpose() * edge.information * eij_vec;
        Eigen::Vector3d bj  = Bij.transpose() * edge.information * eij_vec;

        H.block<3, 3>(3 * i, 3 * i) += Hii;
        H.block<3, 3>(3 * i, 3 * j) += Hij;
        H.block<3, 3>(3 * j, 3 * i) += Hji;
        H.block<3, 3>(3 * j, 3 * j) += Hjj;

        b.segment<3>(3 * i) += bi;
        b.segment<3>(3 * j) += bj;
    }

#else
    ThreadPool pool(std::thread::hardware_concurrency());
    std::mutex mtx;

    for (const Edge& edge : edges) {
        pool.enqueue([&H, &b, &edge, &mtx, this]() {
            u_int64_t i = edge.from;
            u_int64_t j = edge.to;

            const Pose& xi = poses[i];
            const Pose& xj = poses[j];
            const Pose& zij = edge.measurement;

            // Compute error term
            Eigen::Matrix3d Ti = xi.toTransformationMatrix();
            Eigen::Matrix3d Tj = xj.toTransformationMatrix();
            Eigen::Matrix3d Tij = zij.toTransformationMatrix();
            Eigen::Matrix3d eij = Tij.inverse() * Ti.inverse() * Tj;
            Pose eij_pose = fromTransformationMatrix(eij);
            Eigen::Vector3d eij_vec;
            eij_vec << eij_pose.position[0], eij_pose.position[1], eij_pose.orientation;

            Eigen::Matrix2d Ri = Ti.topLeftCorner<2, 2>();
            Eigen::Matrix2d Rij = Tij.topLeftCorner<2, 2>();

            double s = sin(xi.orientation);
            double c = cos(xi.orientation);

            Eigen::Matrix2d dRi;
            dRi << -s, -c,
                    c, -s;

            Eigen::Vector2d dtij;
            dtij << xj.position[0] - xi.position[0], 
                    xj.position[1] - xi.position[1];

            Eigen::Matrix3d Aij = Eigen::Matrix3d::Zero();
            Aij.block<2, 2>(0, 0) = -Rij.transpose() * Ri.transpose();
            Aij.block<2, 1>(0, 2) = Rij.transpose() * dRi.transpose() * dtij;
            Aij(2, 2) = -1;

            Eigen::Matrix3d Bij = Eigen::Matrix3d::Zero();
            Bij.block<2, 2>(0, 0) = Rij.transpose() * Ri.transpose();
            Bij(2, 2) = 1;

            Eigen::Matrix3d Hii = Aij.transpose() * edge.information * Aij;
            Eigen::Matrix3d Hij = Aij.transpose() * edge.information * Bij;
            Eigen::Matrix3d Hji = Hij.transpose();
            Eigen::Matrix3d Hjj = Bij.transpose() * edge.information * Bij;

            Eigen::Vector3d bi = Aij.transpose() * edge.information * eij_vec;
            Eigen::Vector3d bj = Bij.transpose() * edge.information * eij_vec;

            // Protect shared resources H and b using a mutex
            std::lock_guard<std::mutex> lock(mtx);

            H.block<3, 3>(3 * i, 3 * i) += Hii;
            H.block<3, 3>(3 * i, 3 * j) += Hij;
            H.block<3, 3>(3 * j, 3 * i) += Hji;
            H.block<3, 3>(3 * j, 3 * j) += Hjj;

            b.segment<3>(3 * i) += bi;
            b.segment<3>(3 * j) += bj;
        });
    }

    pool.waitForCompletion();
}
#endif

void PoseGraph::optimise(int iterations) {
    int filenum = 1;
    for (int iter = 0; iter < iterations; ++iter) {
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(poses.size() * 3, poses.size() * 3);
        Eigen::VectorXd b = Eigen::VectorXd::Zero(poses.size() * 3);

        computeErrorAndJacobians(H, b);
        H.block<3, 3>(0,0) += Eigen::Matrix3d::Identity();
        // Solve for dx
        Eigen::VectorXd dx = H.ldlt().solve(-b);

        // Update poses and write to file
        std::string filename = "../output/output_" + std::to_string(filenum) + ".txt";
        std::ofstream outFile(filename);
        for (int i = 0; i < poses.size(); ++i) {
            poses[i].position.x() += dx(3 * i);
            poses[i].position.y() += dx(3 * i + 1);
            poses[i].orientation += dx(3 * i + 2);

            outFile << std::round(poses[i].position.x()*1000)/1000 << "," << std::round(poses[i].position.y()*1000)/1000 << "," << std::round(poses[i].orientation*1000)/1000 << std::endl;
        }
        outFile.close();
        std::cout << "Optimising..." << std::endl;
        filenum++;
    }
}

void PoseGraph::importEdges(std::string filename) {
    std::ifstream file(filename);
    std::string line;

    // Skip the header line
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::istringstream iss(line);

        std::string type;
        u_int64_t IDout, IDin;
        double dx, dy, dth, I11, I12, I22, I33, I13, I23;
        Eigen::Matrix3d info_matrix;
        iss >> type >> IDout >> IDin >> dx >> dy >> dth >> I11 >> I12 >> I22 >> I33 >> I13 >> I23;
        info_matrix << I11, I12, I13,
                       I12, I22, I23,
                       I13, I23, I33;
        Pose measurement(dx, dy, dth);
        Edge edge(IDout, IDin, measurement, info_matrix);
        addEdge(edge);
    }
}

void PoseGraph::importPoses(std::string filename) {
    std::ifstream file(filename);
    std::string line;

    // Skip the header line
    std::getline(file, line);

    while (std::getline(file, line)) {
        std::istringstream iss(line);

        std::string type;
        u_int64_t ID;
        double x, y, theta;
        Eigen::Matrix3d info_matrix;
        iss >> type >> ID >> x >> y >> theta;
        Pose pose(x, y, theta);
        addPose(pose, ID);
    }
}
} // namespace PoseGraph