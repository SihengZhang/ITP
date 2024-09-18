#ifndef DISTANCES_SIGNPOST_DATA_STRUCTURE_H
#define DISTANCES_SIGNPOST_DATA_STRUCTURE_H
#include <Eigen/Core>
#include "igl/edges.h"
#include "igl/internal_angles.h"
#include "igl/vertex_triangle_adjacency.h"
#include "igl/boundary_loop.h"
#include "igl/adjacency_list.h"
#include "igl/dijkstra.h"
#include <chrono>
#include <queue>
#include <map>
#include <cmath>

class signpost_data_structure {
public:

    int counter;

    ////data of extrinsic triangulation

    //#V by 3 list of vertex position
    Eigen::MatrixXd V;
    //#F by 3 list of vertex faces
    Eigen::MatrixXi F;

    ////invariant data to connect extrinsic and intrinsic triangulation

    //number of vertices
    int VertexNumber;

    //number of faces;
    int FaceNumber;

    //number of half edges
    int HalfEdgeNumber;

    //#V by 1 list of the sum of angles on each vertex
    Eigen::VectorXd VertexAngleSums;

    //#V by 1 list of fixed zero direction of local tangent polar coordinate on each vertex,
    //represented by an arbitrary adjacent vertex, the direction is from polar vertex to the adjacent vertex
    std::vector<int> ZeroDirection;

    //#V list of the adjacent vertices of each vertex in the extrinsic triangulation, sorted counter-clockwise
    std::vector<std::vector<int>> ExtrinsicAdjacencyList;

    ////data of intrinsic triangulation

    struct HalfEdge {
    public:

        //data
        HalfEdge * twin;
        HalfEdge * next;
        HalfEdge * prev;
        double angle;
        double length;
        int vertex;
        int face;

        //constructor
        HalfEdge(): twin(nullptr), next(nullptr),
        prev(nullptr), angle(-1.), length(-1.),
        vertex(-1), face(-1) {
        }
    };

    //#V by 1 list of an arbitrary edge which source is vertex i
    std::vector<HalfEdge*> VertexIncidentEdge;

    //#F by 1 list of an arbitrary edge which is contained by face i
    std::vector<HalfEdge*> FaceIncidentEdge;

    //map from vertices pair to half edge
    std::map<std::pair<int, int>, HalfEdge*> Edges;



    ////constructor: construct signpost by vertex face adjacency list of the extrinsic mesh
    signpost_data_structure(const Eigen::MatrixXd & Ve, const Eigen::MatrixXi & Fe);

    //// query functions

    bool isFlippable(int source, int target);

    std::vector<int> counterClockwiseIterateNeighborVertices(int source);

    static double oppositeDiagonal(HalfEdge * e);

    std::vector<int> iterateThroughOneSideOfWedge(int a, int b, int c, bool isLeft);

    double getAngleOfWedge(int a, int b, int c, bool isLeft);

    ////operate functions

    //flip one edge, must confirm that the edge is flippable before
    bool flipEdge(int source, int target);

    //must confirm that the wedge is a flexible wedge before
    std::vector<int> flipOutJoint(int a, int b, int c, bool isLeft);

    std::vector<int> makePathGeodesic(std::vector<int> path);

    void iterativelyShortenPath(int source,
                                int target,
                                std::vector<int> & verticesPath,
                                Eigen::MatrixXd & polylineSource,
                                Eigen::MatrixXd & polylineTarget);

    ////track intrinsic edges

    void trackIntrinsicEdges(std::vector<int> verticesPath,
                             Eigen::MatrixXd & polylineSource,
                             Eigen::MatrixXd & polylineTarget);

    void showAllIntrinsicEdges(Eigen::MatrixXd & polylineSource,
                               Eigen::MatrixXd & polylineTarget);





};


#endif //DISTANCES_SIGNPOST_DATA_STRUCTURE_H
