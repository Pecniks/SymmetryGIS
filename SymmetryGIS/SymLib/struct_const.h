#ifndef STRUCT_CONST_H
#define STRUCT_CONST_H

#include <math.h>

#define MIN_VALUE -1000000
#define MAX_VALUE  1000000

#define PI 3.141592654

#define VOX_RADIUS 0.6

#define SPLIT3D_LIMIT 5

namespace Symmetry
{
    struct Point3D
    {
        Point3D()
        {
            x = y = z = 0;
        }

        Point3D(double px, double py, double pz)
        {
            x = px;
            y = py;
            z = pz;
        }

        double x, y, z;
    };

    struct Vector3Di
    {
        Vector3Di()
        {
            x = 0; y = 0; z = 0;
        }

        int x, y, z;
    };

    struct Vector3D
    {
        Vector3D()
        {
            x = 0; y = 0; z = 0;
        }

        double distanceToPoint(Vector3D p2)
        {
            double val = (x - p2.x) * (x - p2.x) + (y - p2.y) * (y - p2.y) + (z - p2.z) * (z - p2.z);
            return sqrt(val);
        }
        double x, y, z;
    };

    struct SymetryPlaneData
    {
        SymetryPlaneData()
        {
            symetry = 0;
        }
        double symetry;
        Vector3D normal;
        int angle;
    };

    struct SymClass
    {
        Vector3D COG;
        int counter;
    };
}//namespace Symmetry
#endif // STRUCT_CONST_H
