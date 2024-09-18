#include "get_dijkstra_edges_path.h"


void get_dijkstra_edges_path(
        const Eigen::MatrixXi & F,
        int source,
        int target,
        Eigen::MatrixXi & Edges) {

    //a set of target vertices, this time we only have one target
    std::set<int> targets;
    targets.insert(target);

    //get adjacency list
    std::vector<std::vector<int>> VV;
    igl::adjacency_list(F, VV);

    //out put of dijkstra
    Eigen::VectorXd min_distance;
    Eigen::VectorXi previous;

    //do dijkstra algorithm to get the list of previous passed vertices
    igl::dijkstra(source, targets, VV, min_distance, previous);

    //backtrack to find the sequence of passed vertices
    std::vector<int> dijkstraPath;
    igl::dijkstra(target, previous, dijkstraPath);

    //number of edges along the path
    int k = (int)dijkstraPath.size() - 1;
    assert(k > 0);

    //get the output matrix
    Edges.resize(k, 2);
    for(int i = 0; i < k; i++) {
        Edges(i, 0) = dijkstraPath[i];
        Edges(i, 1) = dijkstraPath[i + 1];
    }

}