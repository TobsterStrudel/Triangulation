#include "CGALComponents.h"
#include "PolygonTriangulation.h"

int main(int argc, char *argv[]) {

    QApplication a(argc, argv);

    unsigned n = 10;
    Polygon P = generatePolygon(n);
    std::vector<Edge> diagonals;
    diagonals.reserve(n-3);

    std::vector<Point> vertices;
    vertices.reserve(n);

    std::vector<QColor> vertexColors;
    vertexColors.reserve(n);

    triangulatePolygon(P, vertices, vertexColors, diagonals);

    drawPolygonUsingQT(vertices, vertexColors, diagonals, true);

}
