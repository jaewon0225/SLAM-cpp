Pose Graph Optimisation SLAM implementation in C++

Current file format is .graph but any text file that matches the format of the examples in the data directory will work.

Import edges/vertices with PoseGraph::importEdges/Poses function and start optimising the pose graph with PoseGraph::optimise. The user will have to provide the number of iterations for now, but the future plan is to add a threshold to define iteration numbers.

Will require Eigen library to use.