#ifndef ROTATIONALSYMMETRY_H
#define ROTATIONALSYMMETRY_H

#include <vector>

#include "helper_classes/linesegment.h"
#include "helper_classes/voxel.h"
#include "localsymmetry.h"


namespace Symmetry {
    class LineSegmentGraph;
    class Vertex;
    class Edge;
    using LineSegmentPair = std::tuple<LineSegment, LineSegment>;
    using SplitLineSegmentPairs = std::vector<std::vector<std::vector<std::vector<LineSegmentPair>>>>;

    // Class for rotational symmetries.
    class RotationalSymmetry : public LocalSymmetry {
    private:
        // OBJECT VARIABLES
        Point<double> symmetryAxis;             // Symmetry axis point.
        int rotation;                           // Rotation level (1, 2, ..., n).
        std::vector<LineSegment> lineSegments;  // Line segments of the rotational symmetry.
        std::vector<Voxel> voxels;              // Vector of voxels, included in the symmetry.

        // CLASS METHODS
        static SplitLineSegmentPairs splitLineSegments(const std::vector<LineSegment>& lineSegments, const double lengthTolerance, const double angleTolerance);  // Finding simple symmetries.
        static std::vector<LineSegmentGraph> buildLineSegmentGraphs(SplitLineSegmentPairs& pairs);  // Building line segment graphs.
        static std::vector<RotationalSymmetry> findSimpleSymmetries(const std::vector<LineSegmentGraph>& graphs);
    public:
        // CLASS METHODS
        static std::vector<RotationalSymmetry> calculateRotationalSymmetries(const std::vector<Point<float>>& points, const double lengthTolerance, const double angleTolerance);  // Calculating rotational symmetries.
    };

    // Line segment graph class.
    class LineSegmentGraph {
    private:
        std::vector<Vertex> vertices;          // Graph vertices (line segments).
        std::vector<std::vector<bool>> edges;  // 2D matrix of edges.
        int rotation;                          // Rotation level (1, 2, ..., n).
    public:
        LineSegmentGraph(const std::vector<LineSegmentPair>& lineSegmentPairs, const unsigned int rotation);  // Main constructor.
        void buildGraphFromLineSegmentsPairs(const std::vector<LineSegmentPair>& lineSegmentPairs);           // Building a graph from line segment pairs.
    };

    // Line segment graph vertex class.
    class Vertex {
        unsigned int index;
        LineSegment lineSegment;
    public:
        Vertex();
        Vertex(const unsigned int index, const LineSegment& lineSegment);
    };
};

#endif // ROTATIONALSYMMETRY_H
