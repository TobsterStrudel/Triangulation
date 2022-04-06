//
// Created by Ghosh, Anirban on 1/19/22.
//

#ifndef CODE_CGALCOMPONENTS_H
#define CODE_CGALCOMPONENTS_H

#include <vector>
#include <unordered_set>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/point_generators_2.h>
#include <CGAL/random_convex_set_2.h>
#include <CGAL/Timer.h>
#include <CGAL/random_polygon_2.h>
#include <CGAL/Polygon_2.h>

typedef CGAL::Simple_cartesian<double> K;
typedef K::Point_2 Point;
typedef std::list<Point>                         Container;
typedef CGAL::Polygon_2<K, Container>              Polygon;
typedef std::pair<unsigned,unsigned> Edge;

//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//typedef CGAL::Exact_predicates_inexact_constructions_kernel K;

void generatePointsInsideASquare(const unsigned n, const double sizeOfSquare, std::vector<Point> &P) {
    typedef CGAL::Random_points_in_square_2< K::Point_2, CGAL::Creator_uniform_2< double, K::Point_2 > > Point_generator;
    Point_generator g(sizeOfSquare/2);
    std::copy_n(g, n, back_inserter(P));
}

void generateConvexPointSetInsideASquare(const unsigned n, const double sizeOfSquare, std::vector<Point> &P) {
    assert(P.size() == 0);
    typedef CGAL::Random_points_in_square_2<Point, CGAL::Creator_uniform_2< double, Point > > pg;
    CGAL::random_convex_set_2( n, std::back_inserter(P), pg( sizeOfSquare));
}

std::unordered_map<Point,unsigned> createMapOfPoints(const std::vector<Point> &P) {
    std::unordered_map<Point,unsigned> M;
    for(unsigned i = 0; i < P.size(); ++i)
        M[P[i]] = i;
    return M;
}

void getPolygonVertices(const Polygon &P, std::vector<Point> &vertices){
    for (auto vi = P.vertices_begin(); vi != P.vertices_end(); ++vi)
        vertices.emplace_back(*vi);
}

//void getPolygonEdges(const Polygon &P, std::vector<Edge> &polygonEdges) {
//    std::vector<Point> polygonVertices;
//    getPolygonVertices(P,polygonVertices);
//    auto pointsToIdsMap = createMapOfPoints(polygonVertices);
//    for (auto ei = P.edges_begin(); ei != P.edges_end(); ++ei)
//        polygonEdges.emplace_back(std::make_pair( pointsToIdsMap[(*ei).start()], pointsToIdsMap[(*ei).end()]  ) );
//}

Polygon generatePolygon(const unsigned n) {
    Polygon P;
    typedef CGAL::Random_points_in_square_2<Point> pg;
    CGAL::random_polygon_2(n, std::back_inserter(P), pg(300));
    return P;
}

#endif //CODE_CGALCOMPONENTS_H
