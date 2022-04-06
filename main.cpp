#include "CGALComponents.h"
#include "PolygonTriangulation.h"

int main(int argc, char *argv[]) {

    unsigned fails = 0;
    for(unsigned n = 100; n <= 1000; n += 100) {
        std::cout << "n: " << n << "; ";
        for(unsigned i = 0; i < 5; ++i) {
            std::cout << "Run " << i + 1 << " ";
            Polygon P = generatePolygon(n);

            std::vector<Point> polygonVertices;
//            getPolygonVertices(P, polygonVertices);

            std::vector<QColor> vertexColors;
            std::vector<Edge> diagonals;

            triangulatePolygon(P,polygonVertices,vertexColors,diagonals);

            //drawPolygonUsingQT(polygonVertices, vertexColors, diagonals, false);
            if( !isCorrectColoring(P, vertexColors, diagonals) )
                fails++;
        }
        std::cout << std::endl;
    }

    std::cout << "Failure(s): " << fails << std::endl;
    std::cout << "Total number of runs: " << 10 * 5 << std::endl;
    std::cout << "Failure rate: " << (double)fails/50 * 100.0 << " percent.";
}