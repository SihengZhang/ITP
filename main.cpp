#include "get_dijkstra_edges_path.h"
#include "check_mesh_manifold_and_orientation.h"
#include "signpost_data_structure.h"
#include <igl/read_triangle_mesh.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/unproject_onto_mesh.h>
#include <Eigen/Core>
#include <vector>
#include <iostream>

struct State {
    int sourceVertex = 0;
    int targetVertex = 1;
    bool sourceSet = false;
    bool targetSet = false;
} state;

int main(int argc, char *argv[]) {

    // Load input meshes
    Eigen::MatrixXd V;
    Eigen::MatrixXi F, FF;
    igl::read_triangle_mesh(
    (argc>1?argv[1]:"../data/mountain.obj"),V,F);

    //check mesh manifold and reorient mesh
    if(!check_mesh_manifold_and_orientation(F, FF)) {
        std::cout<<"Failed!"<<std::endl;
        return 0;
    }
    std::cout<<"Success!"<<std::endl;
    F = FF;

    // Some function to upsample
    igl::opengl::glfw::Viewer viewer;
    std::cout<<R"(
    C,c  clear all data and reset selected vertices
    D,d  dijkstra algorithm for input path
    F,f  flip edges iteratively to get geodesic
    L,l  lighting)";


    viewer.data().show_lines = true;
    viewer.data().set_mesh(V,F);

    const auto & clear = [&]() {
        state.sourceSet = false;
        state.targetSet = false;
        Eigen::MatrixXd noPoint = Eigen::MatrixXd(0, 3);
        Eigen::MatrixXi noEdge = Eigen::MatrixXi(0, 2);
        viewer.data().set_points(noPoint, Eigen::RowVector3d(0.,0.,0.));
        viewer.data().set_edges(V, noEdge, Eigen::RowVector3d(0., 0., 0.));
        viewer.data().show_lines = true;
    };

    viewer.callback_mouse_down =
            [&](igl::opengl::glfw::Viewer&, int, int)->bool {

                if(!state.sourceSet || !state.targetSet) {

                    // Find the closest point on mesh to mouse position
                    int fid;
                    Eigen::Vector3f bary;
                    if(igl::unproject_onto_mesh(
                            Eigen::Vector2f(viewer.current_mouse_x, viewer.core().viewport(3) - (float)viewer.current_mouse_y),
                            viewer.core().view,
                            viewer.core().proj,
                            viewer.core().viewport,
                            V,F,
                            fid,bary)) {

                        //get id of the closet vertex
                        int maxC;
                        bary.maxCoeff(&maxC);
                        int vid = F(fid,maxC);

                        //set source and target vertices
                        if(!state.sourceSet) {
                            state.sourceVertex = vid;
                            state.sourceSet = true;
                        } else {
                            state.targetVertex = vid;
                            state.targetSet = true;
                        }

                        //show selected vertices
                        viewer.data().add_points(V.row(vid), Eigen::RowVector3d(39. / 256,197. / 256,187. / 256));
                        return true;
                    }
                }
                return false;
            };


    viewer.callback_key_pressed =
            [&](igl::opengl::glfw::Viewer &, unsigned int key, int) {
                Eigen::VectorXi J, I;
                switch(key) {
                    case 'C':
                    case 'c':
                    {
                        clear();
                    }
                    break;
                    case 'D':
                    case 'd':
                    {
                        Eigen::MatrixXi Edges;
                        get_dijkstra_edges_path(F, state.sourceVertex, state.targetVertex, Edges);
                        viewer.data().set_edges(V, Edges, Eigen::RowVector3d(0., 0., 1.));
                        viewer.data().show_lines = false;
                    }
                    break;
                    case 'F':
                    case 'f':
                    {

                        auto * ptr = new signpost_data_structure(V, F);
                        std::vector<int> path;
                        Eigen::MatrixXd polylineSource, polylineTarget;
                        ptr->iterativelyShortenPath(state.sourceVertex, state.targetVertex, path, polylineSource, polylineTarget);

                        //highlight vertices
                        for(auto v : path) {
                            viewer.data().add_points(V.row(v), Eigen::RowVector3d(1, 0, 0));
                        }

                        //highlight polyline
                        viewer.data().add_edges(polylineSource, polylineTarget, Eigen::RowVector3d(1, 0, 0));

                        viewer.data().show_lines = false;

                    }
                    break;
                    case 'l':
                    case 'L':
                    {
                        // Toggle lighting
                        viewer.core().lighting_factor = (float)1.0- viewer.core().lighting_factor;
                    }
                    break;
                    default:
                        return false;
                }
                return true;
            };

  viewer.launch();

  return EXIT_SUCCESS;
}
