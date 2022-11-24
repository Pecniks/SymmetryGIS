#include "LAS/File.h"
#include "pointreader.h"

using namespace Symmetry;


// Reading points from a LAS file.
std::vector<Point<float>> PointReader::readPointsFromLAS(std::string path) {
    LAS::File las(path);                   // Opening a LAS file with the laslib library.
    std::vector<Point<float>> points;      // Resetting previous LAS points.

    // Reading point by point and finding extreme points.
    for (auto lasPoint : las) {
        // Retreiving point coordinates given in centimeters,
        // therefore we convert them to meters and create new points.
        double x = lasPoint.X() / 100.0;
        double y = lasPoint.Y() / 100.0;
        double z = lasPoint.Z() / 100.0;
        points.push_back(Point<float>(x, y, z));
    }

    /******************************************************************************************/
    /******************************************************************************************/
    /******************************************************************************************/
    /******************************************************************************************/

    // Custom points.
//    std::vector<Point<float>> points = {
//        Point<float>(0, 0, 0),
//        Point<float>(2, 0, 0),
//        Point<float>(0, 2, 0),
//        Point<float>(2, 2, 0),
//        Point<float>(2, 4, 0),
//    };

    return points;
}
