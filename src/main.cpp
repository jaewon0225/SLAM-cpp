#include "slam.cpp"

int main() {
    PoseGraph::PoseGraph graph;
    graph.importEdges("../data/INTEL_P_toro_edges.graph");
    graph.importPoses("../data/INTEL_P_toro_vertex.graph");
    graph.optimise(5);
    std::cout << "done" << std::endl;
}