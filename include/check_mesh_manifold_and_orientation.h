#ifndef DISTANCES_CHECK_MESH_MANIFOLD_AND_ORIENTATION_H
#define DISTANCES_CHECK_MESH_MANIFOLD_AND_ORIENTATION_H
#include <Eigen/Core>
#include "igl/is_vertex_manifold.h"
#include "igl/is_edge_manifold.h"
#include "igl/bfs_orient.h"
#include <iostream>
bool check_mesh_manifold_and_orientation(
        const Eigen::MatrixXi & F,
        Eigen::MatrixXi & FF
        );

#endif //DISTANCES_CHECK_MESH_MANIFOLD_AND_ORIENTATION_H
