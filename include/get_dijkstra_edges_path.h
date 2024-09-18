#ifndef DISTANCES_GET_DIJKSTRA_EDGES_PATH_H
#define DISTANCES_GET_DIJKSTRA_EDGES_PATH_H
#include <Eigen/Core>
#include "igl/dijkstra.h"
#include "igl/adjacency_list.h"
#include <vector>
#include <set>

void get_dijkstra_edges_path(
        const Eigen::MatrixXi & F,
        int source,
        int target,
        Eigen::MatrixXi & Edges);

#endif //DISTANCES_GET_DIJKSTRA_EDGES_PATH_H
