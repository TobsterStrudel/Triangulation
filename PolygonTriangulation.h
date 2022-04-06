//
// Created by Ghosh, Anirban on 2/16/22.
//

#ifndef CODE_POLYGONTRIANGULATION_H
#define CODE_POLYGONTRIANGULATION_H

#include <QtWidgets>
#include <QApplication>
#include <QLabel>
#include <QString>
#include <QTranslator>

#include <cstdlib>
#include <vector>

#include "CGALComponents.h"
bool isClipable(const Polygon &P, std::vector<Point> &points, unsigned i);


// FYI: https://doc.cgal.org/latest/Polygon/classCGAL_1_1Polygon__2.html
// The vector allEdges must contain exactly n-3 diagonals

void drawPolygonUsingQT(const std::vector<Point> &polygonVertices,
                        const std::vector<QColor> &vertexColors,
                        const std::vector<Edge> &diagonals, const bool labels) {
    assert( !polygonVertices.empty() );

    const double pointSize = 4; // adjust this value to change the size of the points
    /***************************************************/
    QPicture pi;
    QPainter canvas(&pi);
    canvas.setRenderHint(QPainter::Antialiasing);
    //canvas.setFont(QFont("Times", 12));
    // DO NOT TOUCH THESE Qt STATEMENTS!!!
    /***************************************************/

    canvas.setBrush(Qt::lightGray);

    std::vector<QPointF> verticesForQTPolygon;
    verticesForQTPolygon.reserve(polygonVertices.size());
    for( Point p : polygonVertices )
        verticesForQTPolygon.emplace_back( QPointF(p.x(),p.y() ) );

    canvas.drawPolygon(&verticesForQTPolygon[0], (int)verticesForQTPolygon.size());

    for ( Edge e : diagonals ) {
        QPointF endPointA(polygonVertices[e.first].x(),polygonVertices[e.first].y() ),
                endPointB(polygonVertices[e.second].x(),polygonVertices[e.second].y() );
        canvas.drawLine( endPointA, endPointB );
    }


    unsigned id = 0;
    for ( Point p : polygonVertices ) {

        if( vertexColors[id] == Qt::red) {
            canvas.setBrush(Qt::red); canvas.setPen(Qt::red);
        }
        else if( vertexColors[id] == Qt::darkGreen) {
            canvas.setBrush(Qt::darkGreen); canvas.setPen(Qt::darkGreen);
        }
        else {
            canvas.setBrush(Qt::blue); canvas.setPen(Qt::blue);
        }

        canvas.drawEllipse(QPointF(p.x(), p.y()), pointSize, pointSize);
        if(labels) {
            canvas.setBrush(Qt::black);
            canvas.setPen(Qt::black);
            QPointF textPos(p.x() + 4.0, p.y() + 4.0);
            canvas.drawText(textPos, QString(std::to_string(id).c_str()));
        }
        id++;
    }

    /*********************************************/
    canvas.end();
    auto l = new QLabel;
    l->setStyleSheet("QLabel { background-color : white; color : black; }");
    l->setPicture(pi);
    l->setWindowTitle("Polygon Triangulation");
    l->show();
    // l->showMaximized();
    QApplication::exec();
    // DO NOT TOUCH THESE Qt STATEMENTS!!!
    /***************************************************/
}

bool isCorrectColoring(const Polygon &P, const std::vector<QColor> &vertexColors, const std::vector<Edge> &diagonals){

    std::vector<Point> polygonVertices;
    getPolygonVertices(P,polygonVertices);

    if( polygonVertices.size() != P.size() - 3 )
        return false;

    auto M = createMapOfPoints(polygonVertices);

    for (auto ei = P.edges_begin(); ei != P.edges_end(); ++ei) {
        unsigned p = M[(*ei).start()], q = M[(*ei).end()];
        if( vertexColors[p] == vertexColors[q] )
            return false;
    }

    for (auto e : diagonals) {
        if( vertexColors[e.first] == vertexColors[e.second] )
            return false;
    }

    return true;
}

Point getItem(std::vector<Point> &points, int index){
    if(index == -1){
        return points[points.size()-1];
    }
    if(index >= points.size()){
        return points[index % points.size()];
    }
    else if(index < 0){
        std::cout << "index: " << index << " new index: " << index % points.size() << std::endl;
        return points[index % points.size()];
    }
    else{
        return points[index];
    }
}
bool validEdge(const Polygon &P, std::pair<Point, Point> e){
    int num = 0;

    for(unsigned i = 0; i < P.size(); i++){
        if(CGAL::intersection(P.edge(i), CGAL::Segment_2<K>(e.first, e.second))){
            num++;
        }
    }

    if(num == 4 && P.has_on_positive_side(midpoint(CGAL::Segment_2<K>(e.first, e.second))))
        return true;
    else return false;
}
// triangulate P and put the n-3 diagonals inside the parameter vector 'diagonals'
std::vector<K::Triangle_2> findNeighbors(K::Triangle_2 triangle ,std::vector<K::Triangle_2> &triangles){
    std::vector<K::Triangle_2> newTriangles;
    int count;
    for(int i = 0; i < triangles.size() || newTriangles.size() < 2; i++){
        count = 0;
        for(int j = 0; j < 3; j++){
            for(int k = 0; k < 3; k++){
                if(triangle.vertex(j) == triangles[j].vertex(k)){
                    count++;
                    if(count == 2){
                        newTriangles.emplace_back(triangles[j]);
                    }

                }
            }

        }
    }
}
void triangulatePolygon(const Polygon &P, std::vector<Point> &verticesOfP, std::vector<QColor> &vertexColors, std::vector<Edge> &diagonals) {

    typedef K::Triangle_2 Triangle;

    assert( diagonals.empty() ); // enable this when you are done
    getPolygonVertices(P,verticesOfP); // do not delete, this is needed for drawing
    std::vector<bool> clipability;
    clipability.reserve(P.size());
    std::vector<Triangle> triangles;

    std::vector<Point> points = verticesOfP;
    auto pointsToIdMap = createMapOfPoints(points);//hash map for getting ids from points
    for(unsigned i = 0; points.size() > 3 && i < verticesOfP.size(); i++){
        clipability.emplace_back(isClipable(P, points, i));
    }

    for(unsigned i = 0; points.size() > 3 && i < verticesOfP.size(); i++){
        if(isClipable(P, points, i)){
            diagonals.emplace_back(pointsToIdMap[getItem(points, i - 1)], pointsToIdMap[getItem(points, i + 1)]);
            triangles.emplace_back(Triangle(getItem(points, i-1),getItem(points, i+1),getItem(points, i)));
            points.erase(points.begin() + i);
            i = -1;
        }
    }
    assert( diagonals.size() == P.size() - 3 ); // uncomment when you are done for wellness checking

    std::vector<std::pair<Triangle, std::vector<Triangle>>> triangles2;
    std::vector<Triangle> neighbors;
    neighbors = findNeighbors(triangles[0], triangles);


    vertexColors[pointsToIdMap[triangles[triangles.size()-1].vertex(0)]] = Qt::red;
    std::cout << pointsToIdMap[triangles[triangles.size()-1].vertex(0)] << std::endl;
    vertexColors[pointsToIdMap[triangles[triangles.size()-1].vertex(1)]] = Qt::blue;
    std::cout << pointsToIdMap[triangles[triangles.size()-1].vertex(1)] << std::endl;
    vertexColors[pointsToIdMap[triangles[triangles.size()-1].vertex(2)]] = Qt::darkGreen;
    std::cout << pointsToIdMap[triangles[triangles.size()-1].vertex(2)] << std::endl;
}
bool isClipable(const Polygon &P, std::vector<Point> &points, unsigned i){
    if(CGAL::orientation(getItem(points, i-1), getItem(points, i), getItem(points,i+1)) == CGAL::LEFT_TURN && validEdge(P, std::pair<Point, Point>(getItem(points, i-1), getItem(points, i+1)))){
        return true;
    }
    return false;
}

#endif //CODE_POLYGONTRIANGULATION_H