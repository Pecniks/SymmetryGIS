#include <pch.h>
#include <algorithm>
#include <tuple>

#include <iostream>

#include "helper_classes/tolerance.h"
#include "rotationalsymmetry.h"

using namespace Symmetry;


// CLASS METHODS
// Splitting the line segments.
SplitLineSegmentPairs RotationalSymmetry::splitLineSegments(const std::vector<LineSegment>& lineSegments, const double lengthTolerance, const double angleTolerance) {

    // Creating a 2D vector (each vector in it will contain the
    // tuples with the angle and the two line segments).
    const unsigned int maximumLevelOfRotation = 15;
    const double toleranceRadians = angleTolerance * (M_PI / 180);  // Transforming the angle tolerance from degrees to radians.
    SplitLineSegmentPairs lineSegmentPairs(maximumLevelOfRotation);
    std::vector<std::vector<double>> lineSegmentLengths(maximumLevelOfRotation);

    // Splitting line segments into parts.
    auto splitLineSegments = LocalSymmetry::splitLineSegmentVectorIntoParts(lineSegments, lengthTolerance);

    // Iterating through all line segment vector parts.
    for (std::vector<LineSegment>& part : splitLineSegments) {
        // Checking every possible combination.
        for (unsigned long long i = 0; i < part.size() - 1; i++) {
            for (unsigned long long j = i + 1; j < part.size(); j++) {
                // If the two line segments do not lie on the same height,
                // we move to the next pair as quickly as possible.
                if (!Tolerance::isInTolerance(part[i].getP1().getZ(), part[j].getP1().getZ(), 0.0001)) {
                    continue;
                }

                // Calculating the angle between the two line segments.
                // Smaller angle in the interval of [0°, 180°] is
                // chosen in order to prevent programmer's hair loss.
                const double angle1 = part[i].calculateAngle(part[j]);
                const double angle2 = part[j].calculateAngle(part[i]);
                const double angle = std::min(angle1, angle2);

                // Calculating the rotation level in radians. If the level is not an integer,
                // the angle is not interesting for a potential rotational symmetry.
                // If the rotation level is bigger than the maximum level, we move on as well.
                const double levelFloat = 2.0 / (1 - (angle / M_PI));
                const double innerAngle = (levelFloat - 2) * M_PI / levelFloat;  // Calculation of the inner angle of the polygon.
                const int level = static_cast<int>(levelFloat);
                if (!Tolerance::isInTolerance(angle, innerAngle, toleranceRadians) || levelFloat > maximumLevelOfRotation) {
                    continue;
                }

                // If the current level vector is empty, a new vector of line segment pairs is added.
                // A new line segment pair is added to the first vector of the current vector.
                // Line segment pairs are divided by separate vectors on the second level based by
                // their length (line segment pairs with the same length belong to the same vector).
                if (lineSegmentPairs[level].empty()){
                    lineSegmentPairs[level].push_back(std::vector<std::vector<LineSegmentPair>>());
                    for (int layer = 0; layer < voxelMesh.voxelZ; layer++) {
                        lineSegmentPairs[level][0].push_back(std::vector<LineSegmentPair>());
                    }

                    const int layer = static_cast<int>(part[i].getP1().getZ() / voxelMesh.voxelSideSize);
                    lineSegmentPairs[level][0][layer].push_back(std::make_tuple(part[i], part[j]));
                    lineSegmentLengths[level].push_back(part[i].getLength());

                    continue;
                }

                // Searching for a vector of line segment pairs with the same length as the current line segment.
                int foundIndex = -1;
                for (unsigned long k = 0; k < lineSegmentPairs[level].size(); k++) {
                    const double d1 = lineSegmentLengths[level][k];  // Current vector line segment length.
                    const double d2 = part[i].getLength();           // The current line segment length.

                    // If the two line segment lengths are within a tolerance,
                    // we have found the right vector for our new pair. Yaaaay!!!
                    if (Tolerance::isInTolerance(d1, d2, 0.01)) {
                        foundIndex = k;
                        break;
                    }
                }

                // If there is no vector with the suitable line segment length,
                // a new vector with a new line segment length is added.
                if (foundIndex == -1) {
                    lineSegmentPairs[level].push_back(std::vector<std::vector<LineSegmentPair>>());
                    const int index = lineSegmentPairs[level].size() - 1;

                    for (int layer = 0; layer < voxelMesh.voxelZ; layer++) {
                        lineSegmentPairs[level][index].push_back(std::vector<LineSegmentPair>());
                    }

                    const int layer = static_cast<int>(part[i].getP1().getZ() / voxelMesh.voxelSideSize);
                    lineSegmentPairs[level][index][layer].push_back(std::make_tuple(part[i], part[j]));
                    lineSegmentLengths[level].push_back(part[i].getLength());
                }
                // Otherwise, a line segment pair is added to the beforefound vector.
                else {
                    const int layer = static_cast<int>(part[i].getP1().getZ() / voxelMesh.voxelSideSize);
                    lineSegmentPairs[level][foundIndex][layer].push_back(std::make_tuple(part[i], part[j]));
                }
            }
        }
    }

    // Removing groups that contain too few (less than the rotation level) number of line segment pairs.
    for (unsigned long level = 2; level < maximumLevelOfRotation; level++) {
        for (unsigned long group = 0; group < lineSegmentPairs[level].size(); group++) {
            for (unsigned long layer = 0; layer < lineSegmentPairs[level][group].size(); layer++) {
                // If the layer contains too few items, the whole layer is deleted.
                if (lineSegmentPairs[level][group][layer].size() < level) {
                    lineSegmentPairs[level][group].erase(lineSegmentPairs[level][group].begin() + layer);
                    layer--;
                }
                else {
                    // Getting the number of unique line segments.
                    std::set<LineSegment> lineSegments;
                    for (unsigned long lineSegment = 0; lineSegment < lineSegmentPairs[level][group][layer].size(); lineSegment++) {
                        auto ls1 = std::get<0>(lineSegmentPairs[level][group][layer][lineSegment]);  // First line segment.
                        auto ls2 = std::get<1>(lineSegmentPairs[level][group][layer][lineSegment]);  // Second line segment.

                        // Inserting the both line segments.
                        lineSegments.insert(ls1);
                        lineSegments.insert(ls2);

                        // If there is equal or larger number of line segments
                        // than the current level, we're all good.
                        if (lineSegments.size() >= level) {
                            break;
                        }
                    }

                    // If there is less line segments than the current level of rotation, the layer is removed.
                    if (lineSegments.size() < level) {
                        lineSegmentPairs[level][group].erase(lineSegmentPairs[level][group].begin() + layer);
                        layer--;
                    }
                }
            }

            // If there is an empty group of line segment pairs, it is deleted.
            if (lineSegmentPairs[level][group].empty()) {
                lineSegmentPairs[level].erase(lineSegmentPairs[level].begin() + group);
                group--;
            }
        }
    }

    return lineSegmentPairs;
}

// Building line segment graphs.
std::vector<LineSegmentGraph> RotationalSymmetry::buildLineSegmentGraphs(SplitLineSegmentPairs& pairs) {
    std::vector<LineSegmentGraph> lineSegmentGraphs;

    for (unsigned long level = 4; level < pairs.size(); level++) {
        for (unsigned long group = 0; group < pairs[level].size(); group++) {
            for (unsigned long layer = 0; layer < pairs[level][group].size(); layer++) {
                lineSegmentGraphs.push_back(LineSegmentGraph(pairs[level][group][layer], level));
            }
        }
    }

    return lineSegmentGraphs;
}

// Finding simple rotational symmetries from graphs.
std::vector<RotationalSymmetry> RotationalSymmetry::findSimpleSymmetries(const std::vector<LineSegmentGraph>& graphs) {
    std::vector<RotationalSymmetry> symmetries;

    return symmetries;
}



// CLASS METHODS
// Calculating rotational symmetries.
std::vector<RotationalSymmetry> RotationalSymmetry::calculateRotationalSymmetries(const std::vector<Point<float>>& points, const double lengthTolerance, const double angleTolerance) {
    std::vector<LineSegment> lineSegments = calculateLineSegmentsBetweenPoints(points, lengthTolerance);  // Calculation of line segments betweeen all pairs of points.
    SplitLineSegmentPairs splitLineSegments = RotationalSymmetry::splitLineSegments(lineSegments, lengthTolerance, angleTolerance);  // Splitting the line segments.
    std::vector<LineSegmentGraph> lineSegmentGraphs = buildLineSegmentGraphs(splitLineSegments);  // Building line segment graphs.
    std::vector<RotationalSymmetry> symmetries = findSimpleSymmetries(lineSegmentGraphs);

    return std::vector<RotationalSymmetry>();
}





/**********************************************************************************************************************************************************/
/**********************************************************************************************************************************************************/
/**********************************************************************************************************************************************************/
/**********************************************************************************************************************************************************/
/**********************************************************************************************************************************************************/
// LINESEGMENTGRAPH
// CLASS METHODS
// Main constructor.
LineSegmentGraph::LineSegmentGraph(const std::vector<LineSegmentPair>& lineSegmentPairs, const unsigned int rotation) :
    rotation(rotation)
{
    buildGraphFromLineSegmentsPairs(lineSegmentPairs);  // Building a graph.
}

// Building a graph from line segment pairs.
void LineSegmentGraph::buildGraphFromLineSegmentsPairs(const std::vector<LineSegmentPair>& lineSegmentPairs) {
    // Initializing helper structures for building a graph.
    std::vector<LineSegment> lineSegments;  // Declaring a line segment vector (future vertices).
    std::vector<std::vector<bool>> connections(2 * lineSegmentPairs.size(), std::vector<bool>(2 * lineSegmentPairs.size(), false));  // Initializing helper connection 2D vector (future edges).

    // Iterating through each pair of line segments.
    for (const LineSegmentPair& pair : lineSegmentPairs) {
        // Getting the both line segments from a pair.
        const LineSegment firstLineSegment = std::get<0>(pair);
        const LineSegment secondLineSegment = std::get<1>(pair);

        // Searching whether each separate line segment is already present in the vector.
        auto firstIterator = std::find(lineSegments.begin(), lineSegments.end(), firstLineSegment);
        auto secondIterator = std::find(lineSegments.begin(), lineSegments.end(), secondLineSegment);

        // Checking whether the iterators are at the end.
        // If not, something has been found :)
        bool firstFound = firstIterator != lineSegments.end();
        bool secondFound = secondIterator != lineSegments.end();

        // Getting the indices of the both line segments in the vector.
        auto firstIndex = std::distance(lineSegments.begin(), firstIterator);
        auto secondIndex = std::distance(lineSegments.begin(), secondIterator);

        // If the first line segment has not been found, we add it
        // to the vector and update the first index accordingly.
        if (!firstFound) {
            lineSegments.push_back(firstLineSegment);
            firstIndex = lineSegments.size() - 1;
        }

        // If the second line segment has not been found, we add it
        // to the vector and update the second index accordingly.
        if (!secondFound) {
            lineSegments.push_back(secondLineSegment);
            secondIndex = lineSegments.size() - 1;
        }

        // Setting the connection (relation or edge)
        // between the two line segments to True.
        connections[firstIndex][secondIndex] = true;
        connections[secondIndex][firstIndex] = true;
    }

    // After having completed the unique line segment search,
    // graph vertices are created from those line segments.
    for (unsigned int i = 0; i < lineSegments.size(); i++) {
        vertices.push_back(Vertex(i, lineSegments[i]));
    }

    // After creating graph vertices, edges are set. In this case,
    // edges represent relations between pairs of line segments.
    edges = std::vector<std::vector<bool>>(lineSegments.size(), std::vector<bool>(lineSegments.size(), false));
    for (unsigned int i = 0; i < lineSegments.size(); i++) {
        for (unsigned int j = 0; j < lineSegments.size(); j++) {
            edges[i][j] = connections[i][j];
        }
    }
}



/**********************************************************************************************************************************************************/
/**********************************************************************************************************************************************************/
/**********************************************************************************************************************************************************/
/**********************************************************************************************************************************************************/
/**********************************************************************************************************************************************************/
// VERTEX
// CLASS METHODS
// Main constructor.
Vertex::Vertex(const unsigned int index, const LineSegment& lineSegment) :
    index(index),
    lineSegment(lineSegment)
{}
