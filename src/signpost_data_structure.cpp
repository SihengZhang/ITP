#include "signpost_data_structure.h"

signpost_data_structure::signpost_data_structure(const Eigen::MatrixXd & Ve,
                        const Eigen::MatrixXi & Fe) {

    this->counter = 0;

    //set V and F
    this->V = Ve;
    this->F = Fe;

    //set VertexNumber, FaceNumber and HalfEdgeNumber
    Eigen::MatrixXi E;
    igl::edges(F, E);
    this->VertexNumber = (int)this->V.rows();
    this->FaceNumber = (int)this->F.rows();
    this->HalfEdgeNumber = 2 * (int)E.rows();

    //set VertexAngleSums
    {
        //get internal angles incident on respective corner of all triangles
        Eigen::MatrixXd A;
        igl::internal_angles(V, F, A);

        //get list of adjacency triangles indices and list of corresponding corner indices
        std::vector<std::vector<int>> VF;
        std::vector<std::vector<int>> VFi;
        igl::vertex_triangle_adjacency(this->VertexNumber, F, VF, VFi);

        //set output vector
        this->VertexAngleSums.resize(this->VertexNumber);
        for(int i = 0; i < this->VertexNumber; i++) {
            double angleSum = 0;
            for(int j = 0; j < VF[i].size(); j++) {
                angleSum += A(VF[i][j], VFi[i][j]);
            }
            this->VertexAngleSums(i) = angleSum;
        }
    }


    //set Edges (except the angle)
    {
        //create all half edges (except boundary edges), and set face, vertex, prev and next
        for(int f = 0; f < F.rows(); f++) {
            std::vector<std::pair<int, int>> faceEdges = {std::pair<int, int>(F(f, 0), F(f, 1)),
                                                          std::pair<int, int>(F(f, 1), F(f, 2)),
                                                          std::pair<int, int>(F(f, 2), F(f, 0))};

            for(int i = 0; i < 3; i++) {
                this->Edges[faceEdges[i]] = new HalfEdge();

                this->Edges[faceEdges[i]]->face = f;
                this->Edges[faceEdges[i]]->vertex = faceEdges[i].first;
                this->Edges[faceEdges[i]]->length = (V.row(faceEdges[i].first) - V.row(faceEdges[i].second)).norm();
            }

            for(int i = 0; i < 3; i++) {
                this->Edges[faceEdges[i]]->next = this->Edges[faceEdges[(i + 1) % 3]];
                this->Edges[faceEdges[i]]->prev = this->Edges[faceEdges[(i + 2) % 3]];
            }
        }


        //create boundary edges and set twin
        for(int f = 0; f < F.rows(); f++) {
            std::vector<std::pair<int, int>> faceEdges = {std::pair<int, int>(F(f, 0), F(f, 1)),
                                                          std::pair<int, int>(F(f, 1), F(f, 2)),
                                                          std::pair<int, int>(F(f, 2), F(f, 0))};

            std::vector<std::pair<int, int>> twinEdges = {std::pair<int, int>(F(f, 1), F(f, 0)),
                                                          std::pair<int, int>(F(f, 2), F(f, 1)),
                                                          std::pair<int, int>(F(f, 0), F(f, 2))};

            for(int i = 0; i < 3; i++) {
                if(this->Edges.find(twinEdges[i]) != this->Edges.end()) {
                    this->Edges[faceEdges[i]]->twin = this->Edges[twinEdges[i]];
                } else {
                    this->Edges[twinEdges[i]] = new HalfEdge();

                    this->Edges[twinEdges[i]]->face = -1;
                    this->Edges[twinEdges[i]]->vertex = twinEdges[i].first;
                    this->Edges[twinEdges[i]]->length = (V.row(twinEdges[i].first) - V.row(twinEdges[i].second)).norm();

                    this->Edges[faceEdges[i]]->twin = this->Edges[twinEdges[i]];
                    this->Edges[twinEdges[i]]->twin = this->Edges[faceEdges[i]];
                }
            }
        }
        assert(this->Edges.size() == this->HalfEdgeNumber);


        std::vector<std::vector<int>> L;
        igl::boundary_loop(F, L);

        //only if the mesh have boundaries, set prev and next pointer by igl::boundary_loop()
        for(auto loop : L) {

            //if the boundary loop is not following the half edges direction, reverse it
            if(this->Edges[std::pair<int, int>(loop[0], loop[1])]->face != -1) {
                std::reverse(loop.begin(), loop.end());
            }

            std::pair<int, int> firstHalfEdge =  std::pair<int, int>(loop[0], loop[1]);
            assert(this->Edges[firstHalfEdge]->face == -1);

            //set boundary half edges' next and prev
            for(int i = 0; i < loop.size(); i++) {
                this->Edges[std::pair<int, int>(loop[i], loop[(i + 1) % loop.size()])]->next =
                        this->Edges[std::pair<int, int>(loop[(i + 1) % loop.size()], loop[(i + 2) % loop.size()])];

                this->Edges[std::pair<int, int>(loop[i], loop[(i + 1) % loop.size()])]->prev =
                        this->Edges[std::pair<int, int>(loop[(i - 1 + loop.size()) % loop.size()], loop[i])];
            }
        }
    }//end set Edges

    //set VertexIncidentEdge
    {
        std::vector<std::vector<int>> A;
        igl::adjacency_list(F, A);
        for(int v = 0; v < A.size(); v++) {
            this->VertexIncidentEdge.push_back(this->Edges[std::pair<int, int>(v, A[v][0])]);
        }
        assert(this->VertexIncidentEdge.size() == this->VertexNumber);
    }

    //set FaceIncidentEdge
    {
        for(int f = 0; f < F.rows(); f++) {
            this->FaceIncidentEdge.push_back(this->Edges[std::pair<int, int>(F(f, 0), F(f, 1))]);
        }
        assert(this->FaceIncidentEdge.size() == this->FaceNumber);
    }

    {
        //calculate angle(only works if the three points is in a mesh triangle)
        auto corner = [](
                const Eigen::RowVector3d & x,
                const Eigen::RowVector3d & y,
                const Eigen::RowVector3d & z)
        {
            Eigen::RowVector3d v1 = (x-y).normalized();
            Eigen::RowVector3d v2 = (z-y).normalized();
            // http://stackoverflow.com/questions/10133957/signed-angle-between-two-vectors-without-a-reference-plane
            double s = v1.cross(v2).norm();
            double c = v1.dot(v2);
            return atan2(s, c);
        };

        for(int v = 0; v < V.rows(); v++) {

            //store neighbor vertices
            std::vector<int> neighbors;

            //judge if the vertex is on the boundary
            bool isBoundaryVertex = false;
            int boundaryTarget = -1;

            //counter-clockwise traversal to iterate around a vertex
            HalfEdge * start = VertexIncidentEdge[v];
            HalfEdge * current = start;
            do {
                if(current->face == -1) {
                    isBoundaryVertex = true;
                    boundaryTarget = current->next->vertex;
                }
                neighbors.push_back(current->next->vertex);
                current = current->prev->twin;
            } while(current != start);

            assert(neighbors.size() >= 2);

            //if the vertex is on the boundary, reshuffle the sequence of neighbor vertices
            if(isBoundaryVertex) {
                std::vector<int> newNeighbors;
                int i = 0;
                while(i < neighbors.size()) {
                    if(neighbors[i] == boundaryTarget) break;
                    i++;
                }
                for(int j = i + 1; j < neighbors.size(); j++) {
                    newNeighbors.push_back(neighbors[j]);
                }
                for(int j = 0; j <= i; j++) {
                    newNeighbors.push_back(neighbors[j]);
                }

                assert(newNeighbors.size() == neighbors.size());
                neighbors = newNeighbors;
            }

            //set ExtrinsicAdjacencyList and ZeroDirection
            this->ExtrinsicAdjacencyList.push_back(neighbors);
            this->ZeroDirection.push_back(neighbors[0]);

            //set angle of each half edges from vertex v
            double angleSum = 0;
            for(int i = 0; i < neighbors.size(); i++) {
                this->Edges[std::pair<int, int>(v, neighbors[i])]->angle = angleSum;
                angleSum += 2 * M_PI * corner(V.row(neighbors[i]), V.row(v), V.row(neighbors[(i + 1) % neighbors.size()])) / this->VertexAngleSums(v);
            }
        }
    }
}//end constructor


bool signpost_data_structure::isFlippable(int source, int target) {

    //must be an exist edge
    if(this->Edges.find(std::pair<int, int>(source, target)) == this->Edges.end()) {
        std::cout<<"Not an exist intrinsic edge!"<<std::endl;
        return false;
    }

    HalfEdge * e = this->Edges[std::pair<int, int>(source, target)];
    HalfEdge * t = e->twin;

    //source and target must both have at least 2 degrees
    int sourceDegree = (int)this->counterClockwiseIterateNeighborVertices(source).size();
    int targetDegree = (int)this->counterClockwiseIterateNeighborVertices(target).size();
    if(sourceDegree < 2 || targetDegree < 2) {
        std::cout<<"No enough degree!"<<std::endl;
        return false;
    }

    //the two triangles must form a convex quad
    double eLength = e->length;
    double tLength = t->length;
    double ePrevLength = e->prev->length;
    double eNextLength = e->next->length;
    double tPrevLength = t->prev->length;
    double tNextLength = t->next->length;

    double sourceE = std::acos((eLength * eLength + ePrevLength * ePrevLength - eNextLength * eNextLength) / (2 * eLength * ePrevLength));
    double sourceT = std::acos((tLength * tLength + tNextLength * tNextLength - tPrevLength * tPrevLength) / (2 * tLength * tNextLength));
    double targetE = std::acos((eLength * eLength + eNextLength * eNextLength - ePrevLength * ePrevLength) / (2 * eLength * eNextLength));
    double targetT = std::acos((tLength * tLength + tPrevLength * tPrevLength - tNextLength * tNextLength) / (2 * tLength * tPrevLength));

    if(sourceE + sourceT >= M_PI || targetE + targetT >= M_PI) {
        std::cout<<"Not convex!"<<std::endl;
        return false;
    }

    return true;
}


std::vector<int> signpost_data_structure::counterClockwiseIterateNeighborVertices(int source) {

    //store neighbor vertices
    std::vector<int> neighbors;

    //counter-clockwise traversal to iterate around a vertex
    HalfEdge * start = VertexIncidentEdge[source];
    HalfEdge * current = start;
    do {
        neighbors.push_back(current->next->vertex);
        current = current->prev->twin;
    } while(current != start);

    return neighbors;
}


double signpost_data_structure::oppositeDiagonal(signpost_data_structure::HalfEdge * e) {

    HalfEdge * t = e->twin;

    //get length of 5 relative edges in the quad
    double eLength = e->length;
    double ePrevLength = e->prev->length;
    double eNextLength = e->next->length;
    double tPrevLength = t->prev->length;
    double tNextLength = t->next->length;

    //get the area of the two triangles
    double halfPerimeterE = (eLength + ePrevLength + eNextLength) / 2;
    double areaE = std::sqrt(halfPerimeterE * (halfPerimeterE - eLength) * (halfPerimeterE - ePrevLength) * (halfPerimeterE - eNextLength));
    double halfPerimeterT = (eLength + tPrevLength + tNextLength) / 2;
    double areaT = std::sqrt(halfPerimeterT * (halfPerimeterT - eLength) * (halfPerimeterT - tPrevLength) * (halfPerimeterT - tNextLength));

    //calculate 3 parts of the equation
    double part1 = (ePrevLength * ePrevLength + eNextLength * eNextLength + tPrevLength * tPrevLength + tNextLength * tNextLength - eLength * eLength) / 2;
    double part2 = ((ePrevLength * ePrevLength - eNextLength * eNextLength) * (tPrevLength * tPrevLength - tNextLength * tNextLength)) / (2 * eLength * eLength);
    double part3 = (8 * areaE * areaT) / (eLength * eLength);

    return std::sqrt(part1 + part2 + part3);
}


std::vector<int> signpost_data_structure::iterateThroughOneSideOfWedge(int a, int b, int c, bool isLeft) {

    std::vector<int> result;

    //if a, b, c can not form a wedge, return empty vector
    if(this->Edges.find(std::pair<int, int>(a, b)) == this->Edges.end() ||  this->Edges.find(std::pair<int, int>(b, c)) == this->Edges.end()) {
        return result;
    }

    //start from edge bc, end in ba (not ab!)
    HalfEdge * start = this->Edges[std::pair<int, int>(b, c)];
    HalfEdge * end = this->Edges[std::pair<int, int>(b, a)];

    //two directions
    if(isLeft) {
        HalfEdge * current = start->prev->twin;
        while(current != end) {
            result.push_back(current->next->vertex);
            current = current->prev->twin;
        }
    } else {
        HalfEdge * current = start->twin->next;
        while(current != end) {
            result.push_back(current->next->vertex);
            current = current->twin->next;
        }
    }

    return result;
}


double signpost_data_structure::getAngleOfWedge(int a, int b, int c, bool isLeft) {

    double wedgeAngle = 0;

    std::vector<int> allVertices;

    //all vertices exclude a and c
    std::vector<int> temp = this->iterateThroughOneSideOfWedge(a, b, c, isLeft);

    //set allVertices
    allVertices.push_back(c);
    for(int v : temp) {
        allVertices.push_back(v);
    }
    allVertices.push_back(a);

    for(int i = 1; i < allVertices.size(); i++) {

        double in1 = this->Edges[std::pair<int, int>(b, allVertices[i - 1])]->length;
        double in2 = this->Edges[std::pair<int, int>(b, allVertices[i])]->length;
        double op;

        //important! if the wedge contains boundaries, we let the angle be Pi to avoid flip it
        if(this->Edges.find(std::pair<int, int>(allVertices[i - 1], allVertices[i])) == this->Edges.end()) {
            return M_PI;

        } else {
            op = this->Edges[std::pair<int, int>(allVertices[i - 1], allVertices[i])]->length;
        }


        wedgeAngle += std::acos((in1 * in1 + in2 * in2 - op * op) / (2 * in1 * in2));
    }

    return wedgeAngle;
}


bool signpost_data_structure::flipEdge(int source, int target) {

    if(!isFlippable(source, target)) {
        return false;
    }

    //get all 6 involved half edges
    HalfEdge * e = this->Edges[std::pair<int, int>(source, target)];
    HalfEdge * t = e->twin;
    HalfEdge * ePrev = e->prev;
    HalfEdge * eNext = e->next;
    HalfEdge * tPrev = t->prev;
    HalfEdge * tNext = t->next;

    //get involved vertices and faces
    int newSource = t->prev->vertex;
    int newTarget = e->prev->vertex;
    int eFace = e->face;
    int tFace = t->face;

    //get the length of new edges
    double oppositeLength = oppositeDiagonal(e);

    //remove references of e and t in VertexIncidentEdge and FaceIncidentEdge
    this->VertexIncidentEdge[source] = tNext;
    this->VertexIncidentEdge[target] = eNext;
    this->FaceIncidentEdge[eFace] = ePrev;
    this->FaceIncidentEdge[tFace] = tPrev;

    //delete old e and t
    this->Edges.erase(std::pair<int, int>(source, target));
    this->Edges.erase(std::pair<int, int>(target, source));

    //create new half edges
    this->Edges[std::pair<int, int>(newSource, newTarget)] = e;
    this->Edges[std::pair<int, int>(newTarget, newSource)] = t;

    //modify the reference of old edges
    ePrev->prev = e;
    ePrev->next = tNext;
    eNext->prev = tPrev;
    eNext->next = t;
    tPrev->prev = t;
    tPrev->next = eNext;
    tNext->prev = ePrev;
    tNext->next = e;

    ePrev->face = eFace;
    eNext->face = tFace;
    tPrev->face = tFace;
    tNext->face = eFace;

    //set new edges
    e->prev = tNext;
    e->next = ePrev;
    t->prev = eNext;
    t->next = tPrev;

    e->twin = t;
    t->twin = e;

    e->face = eFace;
    t->face = tFace;

    e->vertex = newSource;
    t->vertex = newTarget;

    e->length = oppositeLength;
    t->length = oppositeLength;

    //calculate and set new signpost angle
    double eRelativeAngle = std::acos((t->length * t->length + tPrev->length * tPrev->length - eNext->length * eNext->length) /
            (2 * t->length * tPrev->length));
    double tRelativeAngle = std::acos((e->length * e->length + ePrev->length * ePrev->length - tNext->length * tNext->length) /
            (2 * e->length * ePrev->length));

    double eAbsoluteAngle = e->twin->next->angle;
    double tAbsoluteAngle = t->twin->next->angle;

    e->angle = eAbsoluteAngle + 2 * M_PI * eRelativeAngle / this->VertexAngleSums(e->vertex);
    t->angle = tAbsoluteAngle + 2 * M_PI * tRelativeAngle / this->VertexAngleSums(t->vertex);

    return true;
}


std::vector<int> signpost_data_structure::flipOutJoint(int a, int b, int c, bool isLeft) {

    auto firstFlippable = [this](int source, const std::vector<int> & verticesList) -> int
    {
        for(int v : verticesList) {
            if(this->isFlippable(source, v)) {
                return v;
            }
        }
        return -1;
    };

    //vertices along the outer arc
    std::vector<int> outerVertices = this->iterateThroughOneSideOfWedge(a, b, c, isLeft);

    //first encountered edge to be flipped, is no edge, return -1
    int toBeFlipped = firstFlippable(b, outerVertices);

    //repeat flip edge and get all outer vertices until no vertex to be flipped
    while(toBeFlipped != -1) {
        this->flipEdge(b, toBeFlipped);
        this->counter++;
        outerVertices = this->iterateThroughOneSideOfWedge(a, b, c, isLeft);
        toBeFlipped = firstFlippable(b, outerVertices);
    }

    //arrange output
    std::vector<int> output;
    output.push_back(a);
    std::reverse(outerVertices.begin(), outerVertices.end());
    for(int v : outerVertices) {
        output.push_back(v);
    }
    output.push_back(c);

    for(int i = 0; i < output.size() - 1; i++) {
        assert(this->Edges.find(std::pair<int, int>(output[i], output[i + 1])) != this->Edges.end());
    }

    return output;
}


std::vector<int> signpost_data_structure::makePathGeodesic(std::vector<int> path) {

    //compare function of the priority queue
    struct Compare {
        bool operator()(const std::tuple<double, std::vector<int>, bool>& a, const std::tuple<double, std::vector<int>, bool>& b) {
            return std::get<0>(a) > std::get<0>(b);
        }
    };

    //if the number of vertices are smaller than 3, the path is already geodesic
    if(path.size() < 3) {
        return path;
    }

    //priority queue
    std::priority_queue<std::tuple<double, std::vector<int>, bool>,
            std::vector<std::tuple<double, std::vector<int>, bool>>,
            Compare> pq;

    //initialize priority queue
    for(int i = 1; i < path.size() - 1; i++) {

        std::vector<int> wedgeVertices = {path[i - 1], path[i], path[i + 1]};

        double leftAngle = this->getAngleOfWedge(path[i - 1], path[i], path[i + 1], true);
        double rightAngle = this->getAngleOfWedge(path[i - 1], path[i], path[i + 1], false);

        if(leftAngle < M_PI) {
            pq.emplace(leftAngle, wedgeVertices, true);
        }
        if(rightAngle < M_PI) {
            pq.emplace(rightAngle, wedgeVertices, false);
        }
    }

    this->counter = 0;


    //all elements in pq have wedge angle smaller than Pi
    while(!pq.empty()) {

        double wedgeAngle = std::get<0>(pq.top());
        std::vector<int> wedgeVertices = std::get<1>(pq.top());
        bool wedgeIsLeft = std::get<2>(pq.top());
        pq.pop();

        //check whether the wedge exist in the path
        int index = -1;
        for(int i = 1; i < path.size() - 1; i++) {
            //matched
            if(path[i - 1] == wedgeVertices[0] && path[i] == wedgeVertices[1] && path[i + 1] == wedgeVertices[2]) {
                index = i;
                break;
            }
        }

        //wedge has been changed
        if(index == -1) {
            continue;
        }

        //angle has been changed
        if(wedgeAngle != this->getAngleOfWedge(wedgeVertices[0], wedgeVertices[1], wedgeVertices[2], wedgeIsLeft)) {
            continue;
        }

        //shorten the wedge
        std::vector<int> newWedgeVertices = this->flipOutJoint(wedgeVertices[0], wedgeVertices[1], wedgeVertices[2], wedgeIsLeft);


        //update path
        std::vector<int> newPath;
        newPath.reserve(path.size() + 10);
        for(int i = 0; i < index - 1; i++) {
            newPath.push_back(path[i]);
        }

        int begin = (int)newPath.size();

        for(int v : newWedgeVertices) {
            newPath.push_back(v);
        }

        int end = (int)newPath.size();
        for(int i = index + 2; i < path.size(); i++) {
            newPath.push_back(path[i]);
        }
        path = newPath;

        for(int i = 0; i < path.size() - 1; i++) {

            if(i >= begin && i < end && i > 0 && i < path.size() - 1) {
                std::vector<int> vertices = {path[i - 1], path[i], path[i + 1]};

                double leftAngle = this->getAngleOfWedge(path[i - 1], path[i], path[i + 1], true);
                double rightAngle = this->getAngleOfWedge(path[i - 1], path[i], path[i + 1], false);

                if(leftAngle < M_PI) {
                    pq.emplace(leftAngle, vertices, true);
                }
                if(rightAngle < M_PI) {
                    pq.emplace(rightAngle, vertices, false);
                }
            }
        }


    }

    return path;
}


void signpost_data_structure::trackIntrinsicEdges(std::vector<int> verticesPath,
                                                  Eigen::MatrixXd & polylineSource,
                                                  Eigen::MatrixXd & polylineTarget) {

    //calculate angle(only works if the three points is in a mesh triangle)
    auto corner = [](
            const Eigen::RowVector3d & x,
            const Eigen::RowVector3d & y,
            const Eigen::RowVector3d & z)
    {
        Eigen::RowVector3d v1 = (x-y).normalized();
        Eigen::RowVector3d v2 = (z-y).normalized();
        // http://stackoverflow.com/questions/10133957/signed-angle-between-two-vectors-without-a-reference-plane
        double s = v1.cross(v2).norm();
        double c = v1.dot(v2);
        return atan2(s, c);
    };

    std::reverse(verticesPath.begin(), verticesPath.end());

    auto * ptr = new signpost_data_structure(V, F);

    std::vector<Eigen::RowVector3d> outputPath;

    //turn intrinsic edges into polyline on extrinsic mesh for visualization
    for(int i = 1; i < verticesPath.size(); i++) {

        //two vertices
        int start = verticesPath[i - 1];
        int end = verticesPath[i];

        //extract intrinsic edge
        signpost_data_structure::HalfEdge * he = this->Edges[std::pair<int, int>(start, end)];

        //signpost data of the start vertex
        double angleSum = this->VertexAngleSums(start);
        std::vector<int> adjacencyList = this->ExtrinsicAdjacencyList[start];


        double accumulatedAngle = he->angle * angleSum / (2 * M_PI);

        int v = 0;
        while(v < adjacencyList.size()) {

            int current = adjacencyList[v];
            int next = adjacencyList[(v + 1) % (int)adjacencyList.size()];
            double innerAngle = corner(this->V.row(current), this->V.row(start), this->V.row(next));

            if(accumulatedAngle < innerAngle) {
                break;
            }

            accumulatedAngle -= innerAngle;
            v++;
        }

        //residual distance to travel
        double distanceToGo = he->length;

        double relativeAngle = accumulatedAngle;
        int A = start;
        int B = adjacencyList[v];
        int C = adjacencyList[(v + 1) % (int)adjacencyList.size()];
        Eigen::RowVector3d positionP = this->V.row(A);

        while(distanceToGo > 0) {

            outputPath.push_back(positionP);

            Eigen::RowVector3d positionA = this->V.row(A);
            Eigen::RowVector3d positionB = this->V.row(B);
            Eigen::RowVector3d positionC = this->V.row(C);

            double thresholdAngle = corner(positionC, positionP, positionB);

            if(relativeAngle <= thresholdAngle) {
                double angleB = corner(positionA, positionB, positionC);
                double angleN = M_PI - relativeAngle - angleB;
                double PB = (positionP - positionB).norm();
                double PN = PB * std::sin(angleB) / std::sin(angleN);
                double NB = PB * std::sin(relativeAngle) / std::sin(angleN);
                Eigen::RowVector3d positionN = positionB + (positionC - positionB).normalized() * NB;

                distanceToGo -= PN;
                relativeAngle = M_PI - angleN;
                A = C;
                if(ptr->Edges.find(std::pair<int, int>(A, B)) == ptr->Edges.end()) break;
                C = ptr->Edges[std::pair<int, int>(A, B)]->prev->vertex;
                positionP = positionN;
            } else {
                double angleA = corner(positionB, positionA, positionC);
                double angleN = relativeAngle - angleA;
                double PA = (positionP - positionA).norm();
                double PN = PA * std::sin(angleA) / std::sin(angleN);
                double NA = PA * std::sin(M_PI - relativeAngle) / std::sin(angleN);
                Eigen::RowVector3d positionN = positionA + (positionC - positionA).normalized() * NA;

                distanceToGo -= PN;
                relativeAngle = angleN;
                B = C;
                if(ptr->Edges.find(std::pair<int, int>(A, B)) == ptr->Edges.end()) break;
                C = ptr->Edges[std::pair<int, int>(A, B)]->prev->vertex;
                positionP = positionN;
            }

        }
    }



    outputPath.emplace_back(V.row(verticesPath[verticesPath.size() - 1]));

    int n = (int)outputPath.size();

    polylineSource.resize(n - 1, 3);
    polylineTarget.resize(n - 1, 3);

    for(int i = 0; i < n - 1; i++) {
        polylineSource.row(i) = outputPath[i];
        polylineTarget.row(i) = outputPath[i + 1];
    }

}


void signpost_data_structure::showAllIntrinsicEdges(Eigen::MatrixXd &polylineSource, Eigen::MatrixXd &polylineTarget) {

    //calculate angle(only works if the three points is in a mesh triangle)
    auto corner = [](
            const Eigen::RowVector3d & x,
            const Eigen::RowVector3d & y,
            const Eigen::RowVector3d & z)
    {
        Eigen::RowVector3d v1 = (x-y).normalized();
        Eigen::RowVector3d v2 = (z-y).normalized();
        // http://stackoverflow.com/questions/10133957/signed-angle-between-two-vectors-without-a-reference-plane
        double s = v1.cross(v2).norm();
        double c = v1.dot(v2);
        return atan2(s, c);
    };

    auto * ptr = new signpost_data_structure(V, F);


    std::vector<Eigen::RowVector3d> outputPath;
    std::vector<Eigen::RowVector3d> outputSource;
    std::vector<Eigen::RowVector3d> outputTarget;

    //turn intrinsic edges into polyline on extrinsic mesh for visualization
    for(auto iter : this->Edges) {

        //two vertices
        int start = iter.first.first;
        int end = iter.first.second;

        //extract intrinsic edge
        signpost_data_structure::HalfEdge * he = this->Edges[std::pair<int, int>(start, end)];

        //signpost data of the start vertex
        double angleSum = this->VertexAngleSums(start);
        std::vector<int> adjacencyList = this->ExtrinsicAdjacencyList[start];


        double accumulatedAngle = he->angle * angleSum / (2 * M_PI);

        int v = 0;
        while(v < adjacencyList.size()) {

            int current = adjacencyList[v];
            int next = adjacencyList[(v + 1) % (int)adjacencyList.size()];
            double innerAngle = corner(this->V.row(current), this->V.row(start), this->V.row(next));

            if(accumulatedAngle < innerAngle) {
                break;
            }

            accumulatedAngle -= innerAngle;
            v++;
        }

        //residual distance to travel
        double distanceToGo = he->length;

        double relativeAngle = accumulatedAngle;
        int A = start;
        int B = adjacencyList[v];
        int C = adjacencyList[(v + 1) % (int)adjacencyList.size()];
        Eigen::RowVector3d positionP = this->V.row(A);

        while(distanceToGo > 0) {

            outputPath.push_back(positionP);

            Eigen::RowVector3d positionA = this->V.row(A);
            Eigen::RowVector3d positionB = this->V.row(B);
            Eigen::RowVector3d positionC = this->V.row(C);

            double thresholdAngle = corner(positionC, positionP, positionB);

            if(relativeAngle <= thresholdAngle) {
                double angleB = corner(positionA, positionB, positionC);
                double angleN = M_PI - relativeAngle - angleB;
                double PB = (positionP - positionB).norm();
                double PN = PB * std::sin(angleB) / std::sin(angleN);
                double NB = PB * std::sin(relativeAngle) / std::sin(angleN);
                Eigen::RowVector3d positionN = positionB + (positionC - positionB).normalized() * NB;

                distanceToGo -= PN;
                relativeAngle = M_PI - angleN;
                A = C;
                if(ptr->Edges.find(std::pair<int, int>(A, B)) == ptr->Edges.end()) break;
                C = ptr->Edges[std::pair<int, int>(A, B)]->prev->vertex;
                positionP = positionN;
            } else {
                double angleA = corner(positionB, positionA, positionC);
                double angleN = relativeAngle - angleA;
                double PA = (positionP - positionA).norm();
                double PN = PA * std::sin(angleA) / std::sin(angleN);
                double NA = PA * std::sin(M_PI - relativeAngle) / std::sin(angleN);
                Eigen::RowVector3d positionN = positionA + (positionC - positionA).normalized() * NA;

                distanceToGo -= PN;
                relativeAngle = angleN;
                B = C;
                if(ptr->Edges.find(std::pair<int, int>(A, B)) == ptr->Edges.end()) break;
                C = ptr->Edges[std::pair<int, int>(A, B)]->prev->vertex;
                positionP = positionN;
            }

        }
    }

    for(int i = 0; i < outputPath.size() - 1; i++) {
        outputSource.push_back(outputPath[i]);
        outputTarget.push_back(outputPath[i + 1]);
    }

    polylineSource.resize((int)outputSource.size(), 3);
    polylineTarget.resize((int)outputTarget.size(), 3);

    for(int i = 0; i < (int)outputSource.size() - 1; i++) {
        polylineSource.row(i) = outputSource[i];
    }

    for(int i = 0; i < (int)outputTarget.size() - 1; i++) {
        polylineTarget.row(i) = outputTarget[i];
    }

}


void signpost_data_structure::iterativelyShortenPath(int source, int target, std::vector<int> &verticesPath,
                                                     Eigen::MatrixXd &polylineSource, Eigen::MatrixXd &polylineTarget) {
    //get dijkstra path
    std::vector<int> dijkstraPath;
    {
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

        igl::dijkstra(target, previous, dijkstraPath);
    }


    auto start = std::chrono::high_resolution_clock::now();

    //run edge flip algorithm to get the sequence of vertices in intrinsic triangulation
    verticesPath = this->makePathGeodesic(dijkstraPath);

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);

    std::cout<<"iterations: "<<this->counter<<std::endl;
    std::cout<<"milliseconds: "<<duration.count()<<std::endl;

    //track intrinsic edges path on extrinsic mesh and output polyline
    this->trackIntrinsicEdges(verticesPath, polylineSource, polylineTarget);
}
