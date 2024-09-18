#include "check_mesh_manifold_and_orientation.h"

bool check_mesh_manifold_and_orientation(
        const Eigen::MatrixXi & F,
        Eigen::MatrixXi & FF) {

    //check vertex manifold;
    Eigen::Matrix<bool, -1, -1> B;
    if(igl::is_vertex_manifold(F, B)) {
        std::cout<<"this mesh is vertex manifold"<<std::endl;
    } else {
        std::cout<<"this mesh is not vertex manifold"<<std::endl;
        return false;
    }

    //check edge manifold
    if(igl::is_edge_manifold(F)) {
        std::cout<<"this mesh is edge manifold"<<std::endl;
    } else {
        std::cout<<"this mesh is not edge manifold"<<std::endl;
        return false;
    }

    //check mesh orientation
    Eigen::VectorXi C;
    igl::bfs_orient(F, FF, C);
    if(C.maxCoeff() == 0) {
        std::cout<<"this mesh is orientable"<<std::endl;
    } else {
        std::cout<<"this mesh is not orientable"<<std::endl;
        return false;
    }

    return true;
}