#include <pch.h>
#include <algorithm>
#include <set>

#include "LAS/Data/nVector.h"
#include "helper_classes/tolerance.h"
#include "reflectionsymmetry.h"

using namespace Symmetry;



// PRIVATE CLASS METHODS
// Removing voxels without their pairs.
void ReflectionSymmetry::removeVoxelsWithoutPairs(std::vector<ReflectionSymmetry>& reflectionSymmetries) {
    // Checking each reflection symmetry.
    for (ReflectionSymmetry& symmetry : reflectionSymmetries) {
        // Checking all voxels in the symmetry.
        for (unsigned long long i = 0; i < symmetry.voxels.size(); i++) {
            // Getter of the current voxel and calculation of the projection point to the symmetry plane.
            const Voxel& v = symmetry.voxels[i];
            const Point<float> voxelPoint((v.getX() + 0.5f) * voxelMesh.voxelSideSize, (v.getY() + 0.5f) * voxelMesh.voxelSideSize, (v.getZ() + 0.5f) * voxelMesh.voxelSideSize);
            const Point<float> projection = symmetry.plane.calculateProjectionPoint(voxelPoint, voxelMesh);

            // Calculation of the opposite voxel across the symmetry plane.
            LAS::Data::Vector3d vector(projection.getX() - voxelPoint.getX(), projection.getY() - voxelPoint.getY(), projection.getZ() - voxelPoint.getZ());
            const Point<float> oppositePoint = voxelPoint + vector * 2.0;
            const int oppositeX = (int)floor((oppositePoint.getX() / voxelMesh.voxelSideSize) - voxelMesh.minX);
            const int oppositeY = (int)floor((oppositePoint.getY() / voxelMesh.voxelSideSize) - voxelMesh.minY);
            const int oppositeZ = (int)floor((oppositePoint.getZ() / voxelMesh.voxelSideSize) - voxelMesh.minZ);
            const Voxel opposite(oppositeX, oppositeY, oppositeZ);

            auto it = std::find(symmetry.voxels.begin(), symmetry.voxels.end(), opposite);  // Searching whether the opposite voxel is a part of symmetry.

            // If a voxel that should be opposite is not a part of the symmetry,
            // the current voxel is removed from the list.
            if (it == symmetry.voxels.end()) {
                symmetry.voxels.erase(symmetry.voxels.begin() + i);
            }
        }
    }
}

// Calculating the percent of interesting line segment sections.
double ReflectionSymmetry::calculateInterestingPercentInLineSegment(const LineSegment& lineSegment) {
    // Calculating the number of segments.
    int interestingSegments = 0;
    const int segments = (int)(10 * lineSegment.getLength());
    auto& voxels = LocalSymmetry::getVoxels();

    // Calculating the vector from P1 to P2.
    Point<float> p1 = lineSegment.getP1();
    Point<float> p2 = lineSegment.getP2();
    LAS::Data::Vector3d difference = p2 - p1;

    // Marching on the line segment and calculating
    // interesting line segment sections.
    for (int i = 0; i < segments; i++) {
        double factor = (double)i / segments;
        LAS::Data::Vector3d p = LAS::Data::Vector3d(
            p1.getX() + factor * difference.x,
            p1.getY() + factor * difference.y,
            p1.getZ() + factor * difference.z
        );

        // Calculating a voxel position.
        const int x = (int)floor((p.x - voxelMesh.minX) / voxelMesh.voxelSideSize);
        const int y = (int)floor((p.y - voxelMesh.minY) / voxelMesh.voxelSideSize);
        const int z = (int)floor((p.z - voxelMesh.minZ) / voxelMesh.voxelSideSize);

        // If a voxel is interesting, the number of
        // interesting sections is incremented.
        if (voxels[z][y][x].getInteresting()) {
            interestingSegments++;
        }
    }

    return (double)interestingSegments / segments;
}

// Finding simple symmetries.
std::vector<ReflectionSymmetry> ReflectionSymmetry::findSimpleSymmetries(const std::vector<LineSegment>& lineSegments, const double tolerance) {
    std::vector<ReflectionSymmetry> symmetries;  // Symmetry vector.

    // Splitting line segments into parts.
    auto splitLineSegments = ReflectionSymmetry::splitLineSegmentVectorIntoParts(lineSegments, tolerance);

    // Iterating through all line segment vector parts.
    for (std::vector<LineSegment>& part : splitLineSegments) {
        // Checking every possible combination.
        for (unsigned long long i = 0; i < part.size() - 1; i++) {
            for (unsigned long long j = i + 1; j < part.size(); j++) {
                ReflectionSymmetry* symmetry = ReflectionSymmetry::calculateSimpleReflectionSymmetry(part[i], part[j], tolerance);
                if (symmetry != nullptr) {
                    symmetries.push_back(*symmetry);
                }
            }
        }
    }

    return symmetries;
}

// Splitting a line segment vector into parts, where the lenghts of line segments is below the tolerance.
std::vector<std::vector<LineSegment>> ReflectionSymmetry::splitLineSegmentVectorIntoParts(
    const std::vector<LineSegment>& lineSegments,
    const double tolerance
)
{
    // Creating a vector and pushing a first part.
    std::vector<std::vector<LineSegment>> splitLineSegments;
    splitLineSegments.push_back(std::vector<LineSegment>());

    // Iterating through all line segments.
    for (unsigned long long i = 0; i < lineSegments.size(); i++) {
        // If no elements in vector, a first line segment is added.
        if (splitLineSegments.back().size() == 0) {
            splitLineSegments.back().push_back(lineSegments[i]);
        }
        // If a line segment length is inside of the allowed tolerance,
        // it is added to the same part as the previous one.
        else if (
            Tolerance::isInTolerance(
                splitLineSegments.back()[0].getLength(),
                lineSegments[i].getLength(),
                tolerance
            )
        )
        {
            splitLineSegments.back().push_back(lineSegments[i]);
        }
        // If a line segment length is outside of the allowed tolerance,
        // a new part is created, line segment is added to the new part.
        else {
            // If the last part contains only one line
            // segment, there will be no symmetry.
            if (splitLineSegments.back().size() == 1) {
                splitLineSegments.pop_back();
            }

            splitLineSegments.push_back(std::vector<LineSegment>());
            splitLineSegments.back().push_back(lineSegments[i]);
        }
    }

    // If the last part contains only one line
    // segment, there will be no symmetry.
    if (splitLineSegments.back().size() == 1) {
        splitLineSegments.pop_back();
    }

    return splitLineSegments;
}

// Checking whether two line segments represent a reflection symmetry.
ReflectionSymmetry * ReflectionSymmetry::calculateSimpleReflectionSymmetry(LineSegment& l1, LineSegment& l2, const double tolerance) {
    // At first, we have to connect the two line segments.
    LineSegment conn1(l1.getP1(), l2.getP1());  // Making a first connection between line segments.
    LineSegment conn2(l1.getP2(), l2.getP2());  // Making a second connection between line segments.

    // As we only search symmetries that lie in the same layer,
    // NULL pointer is returned if the two Z coordinates are not equal.
    if (!Tolerance::isInTolerance(l1.getP1().getZ(), l2.getP1().getZ(), tolerance)) {
        return nullptr;
    }

    // If the connections intersect with each other, it is necessary
    // to reconect the both line segments with other two points.
    if (LineSegment::doLineSegmentsIntersect(conn1, conn2, tolerance, true)) {
        conn1.setP1(l1.getP1());
        conn1.setP2(l2.getP2());
        conn2.setP1(l1.getP2());
        conn2.setP2(l2.getP1());
    }

    // Calculating center points of the both line segments.
    Point<float> center1 = conn1.getCenterPoint();
    Point<float> center2 = conn2.getCenterPoint();

    // Calculating vectors.
    LAS::Data::Vector3d axis = (center1 - center2).normalize();
    LAS::Data::Vector3d V1 = (conn1.getP1() - conn1.getP2()).normalize();
    LAS::Data::Vector3d V2 = (conn2.getP1() - conn2.getP2()).normalize();

    // If axis cannot be calculated, there is no symmetry.
    if (std::isnan(axis.x) || std::isnan(axis.y) || std::isnan(axis.z)) {
        return nullptr;
    }

    // Calculating the angles between the center line segment and the connections.
    double angle1 = std::acos(axis.dot(V1));
    double angle2 = std::acos(axis.dot(V2));

    // If both angles are equal (with tolerance) to PI/2, a symmetry is found.
    if ((std::isnan(angle1) || Tolerance::isInTolerance(angle1, M_PI / 2, tolerance)) &&
        (std::isnan(angle2) || Tolerance::isInTolerance(angle2, M_PI / 2, tolerance))
    )
    {
        // Creating a plane and a vector of line segments.
        LAS::Data::Vector3d point = center1.toVec();
        if (point.x == 0 && point.y == 0 && point.z == 0) {
            point = center2.toVec();
        }
        Plane plane(point, axis, LAS::Data::Vector3d(0, 0, 1));
        std::vector<LineSegment> lineSegments { l1, l2 };

        return new ReflectionSymmetry(plane, lineSegments);
    }

    return nullptr;
}

// Merging reflection symmetries.
std::vector<ReflectionSymmetry> ReflectionSymmetry::mergeSymmetries(
    std::vector<ReflectionSymmetry>& reflectionSymmetries,
    const double tolerance
)
{
    // Vector of vectors of symmetries with certain full quotients:
    // { [0, 0.25], (0.25, 0.50], (0.50, 0.75], (0.75, 1.00] }.
    std::vector<std::vector<ReflectionSymmetry>> mergedSymmetries(4);

    // Merging the found symmetries by Z axis and center point.
    for (ReflectionSymmetry& symmetry : reflectionSymmetries) {
        // Selecting the index according to the full quotient.
        int index = -1;
        if (symmetry.getLineSegments()[0].getFullQuotient() <= 0.25) {
            index = 0;
        }
        else if (symmetry.getLineSegments()[0].getFullQuotient() <= 0.5) {
            index = 1;
        }
        else if (symmetry.getLineSegments()[0].getFullQuotient() <= 0.75) {
            index = 2;
        }
        else {
            index = 3;
        }

        // Searching for a potential symmetry in the list, where X and
        // Y coordinates are the same as in the current symmetry, the
        // symmetry level is also the same.
        auto itr = std::find_if(
            mergedSymmetries[index].begin(),
            mergedSymmetries[index].end(),
            [&symmetry,& tolerance](ReflectionSymmetry& curSymmetry) {
                Plane p1 = symmetry.getPlane();
                Plane p2 = curSymmetry.getPlane();

                return (
                    (
                        Tolerance::isInTolerance(p1.getA(), p2.getA(), tolerance) &&
                        Tolerance::isInTolerance(p1.getB(), p2.getB(), tolerance) &&
                        Tolerance::isInTolerance(p1.getC(), p2.getC(), tolerance) &&
                        Tolerance::isInTolerance(p1.getD(), p2.getD(), tolerance)
                    )
                    ||
                    (
                        Tolerance::isInTolerance(p1.getA(), -p2.getA(), tolerance) &&
                        Tolerance::isInTolerance(p1.getB(), -p2.getB(), tolerance) &&
                        Tolerance::isInTolerance(p1.getC(), -p2.getC(), tolerance) &&
                        Tolerance::isInTolerance(p1.getD(), -p2.getD(), tolerance)
                    )
                );
            }
        );

        // Adding the symmetry to the list if not exists.
        if (itr == mergedSymmetries[index].end()) {
            mergedSymmetries[index].push_back(symmetry);
        }
        // Updating the Symmetry object with new limits.
        else {
           auto i = std::distance(mergedSymmetries[index].begin(), itr);
            mergedSymmetries[index][i] &= symmetry;
        }
    }

    std::vector<ReflectionSymmetry> flatSymmetries = mergedSymmetries[3];

    // Removing voxels that don't have pairs.
    removeVoxelsWithoutPairs(flatSymmetries);

    // Sorting reflection symmetries by line segment count in each symmetry.
    std::sort(
        flatSymmetries.begin(),
        flatSymmetries.end(),
        [](ReflectionSymmetry& s1, ReflectionSymmetry& s2) {
            return s1.getLineSegmentCount() > s2.getLineSegmentCount();
        }
    );

    return flatSymmetries;
}

// Getting voxels for each reflection symmetry.
void ReflectionSymmetry::getVoxelsInSymmetries(std::vector<ReflectionSymmetry>& reflectionSymmetries) {
    // Getting voxels in each reflection symmetry.
    for (unsigned long long i = 0; i < reflectionSymmetries.size(); i++) {
        reflectionSymmetries[i].voxels.clear();  // Clearing the previous possible artefacts in the list. Better safe than sorry.

        // Adding voxels according to reflection symmetry line segments.
        for (const LineSegment& ls : reflectionSymmetries[i].lineSegments) {
            // First line segment point.
            {
                // Getting the line segment point and calculating the voxel coordinates.
                const Point<float> p = ls.getP1();
                const int x = (int)floor((p.getX() - voxelMesh.minX) / voxelMesh.voxelSideSize);
                const int y = (int)floor((p.getY() - voxelMesh.minY) / voxelMesh.voxelSideSize);
                const int z = (int)floor((p.getZ() - voxelMesh.minZ) / voxelMesh.voxelSideSize);

                // Creating a new voxel.
                Voxel v(x, y, z, true, true);
                v.setInSymmetry(true);

                // If the voxel is not already present in the symmetry, it is added to the list.
                if (std::find(reflectionSymmetries[i].voxels.begin(), reflectionSymmetries[i].voxels.end(), v) == reflectionSymmetries[i].voxels.end()) {
                    reflectionSymmetries[i].voxels.push_back(Voxel(x, y, z, true, true));
                }
            }
            // Second line segment point.
            {
                // Getting the line segment point and calculating the voxel coordinates.
                Point<float> p = ls.getP2();
                const int x = (int)floor((p.getX() - voxelMesh.minX) / voxelMesh.voxelSideSize);
                const int y = (int)floor((p.getY() - voxelMesh.minY) / voxelMesh.voxelSideSize);
                const int z = (int)floor((p.getZ() - voxelMesh.minZ) / voxelMesh.voxelSideSize);

                // Creating a new voxel.
                Voxel v(x, y, z, true, true);
                v.setInSymmetry(true);

                // If the voxel is not already present in the symmetry, it is added to the list.
                if (std::find(reflectionSymmetries[i].voxels.begin(), reflectionSymmetries[i].voxels.end(), v) == reflectionSymmetries[i].voxels.end()) {
                    reflectionSymmetries[i].voxels.push_back(Voxel(x, y, z, true, true));
                }
            }
        }
    }
}

// Adding interesting voxels back to symmetry (if in symmetry).
void ReflectionSymmetry::addInterestingVoxelsToSymmetry(std::vector<ReflectionSymmetry>& reflectionSymmetries) {
    const auto& voxels = LocalSymmetry::getNormalisedVoxelVector();  // Getting the 3D voxel vector.

    // Adding interesting voxels for each reflection symmetry.
    for (ReflectionSymmetry& symmetry : reflectionSymmetries) {
        std::vector<std::tuple<double, Point<float>, Point<float>>> distances;  // Distances from the plane to interesting voxels and plane projection points.

        // Checking voxels by each layer.
        for (int z = 0; z < voxelMesh.voxelZ; z++) {
            distances.clear();  // Clearing distances from the previous iteration.

            for (int y = 0; y < voxelMesh.voxelY; y++) {
                for (int x = 0; x < voxelMesh.voxelX; x++) {
                    if (symmetry.plane.getD() == 12 && x == 6 && y == 1 && z == 0) {
                        int xxx = 888;
                    }

                    // Getting a voxel from the mesh according to indices and checking whether it is already present in the list.
                    const Voxel& v = voxels[z][y][x];
                    bool exists = std::find(symmetry.voxels.begin(), symmetry.voxels.end(), v) != symmetry.voxels.end();

                    // If the voxel is (super) interesting and is not a part of the symmetry,
                    // it is checked if the voxel has its counterpart voxel on the other side
                    // of the symmetry plane.
                    if ((v.getInteresting() || v.getSuperInteresting()) && !exists) {
                        // Calculating voxel indices and the plane projection point.
                        const float voxelX = (x + 0.5f) * voxelMesh.voxelSideSize + voxelMesh.minX;
                        const float voxelY = (y + 0.5f) * voxelMesh.voxelSideSize + voxelMesh.minY;
                        const float voxelZ = (z + 0.5f) * voxelMesh.voxelSideSize + voxelMesh.minZ;
                        const Point<float> voxelPoint(voxelX, voxelY, voxelZ);
                        const Point<float> projection = symmetry.plane.calculateProjectionPoint(voxelPoint, voxelMesh);

                        // Calculation projection point voxel indices.
                        const int xp = (int)floor((projection.getX() - voxelMesh.minX) / voxelMesh.voxelSideSize);  // X index.
                        const int yp = (int)floor((projection.getY() - voxelMesh.minY) / voxelMesh.voxelSideSize);  // Y index.
                        const int zp = (int)floor((projection.getZ() - voxelMesh.minZ) / voxelMesh.voxelSideSize);  // Z index.

                        // If the projection point lies within the same voxel, we add the voxel to the list without second thoughts.
                        if (x == xp && y == yp && z == zp) {
                            symmetry.voxels.push_back(Voxel(x, y, z, true));
                        }
                        // If the symmetry plane is vertical and the symmetry plane goes directly
                        // through a voxel edge, two voxels (current and next) are added to the list.
                        else if (x == xp - 1 && Tolerance::isInTolerance(xp * voxelMesh.voxelSideSize, projection.getX(), 0.0001) && symmetry.plane.getA() == 1) {
                            if ((voxels[z][y][x].getSuperInteresting() || voxels[z][y][x].getInteresting()) &&
                                (voxels[z][y][x + 1].getSuperInteresting() || voxels[z][y][x + 1].getInteresting())
                            )
                            {
                                symmetry.voxels.push_back(Voxel(x, y, z, true));
                                symmetry.voxels.push_back(Voxel(x + 1, y, z, true));
                            }
                        }
                        // If the symmetry plane is horizontal and the symmetry plane goes directly
                        // through a voxel edge, two voxels (current and next) are added to the list.
                        else if (y == yp - 1 && Tolerance::isInTolerance(yp * voxelMesh.voxelSideSize, projection.getY(), 0.0001) && symmetry.plane.getA() == 0) {
                            if ((voxels[z][y][x].getSuperInteresting() || voxels[z][y][x].getInteresting()) &&
                                (voxels[z][y + 1][x].getSuperInteresting() || voxels[z][y + 1][x].getInteresting())
                            )
                            {
                                symmetry.voxels.push_back(Voxel(x, y, z, true));
                                symmetry.voxels.push_back(Voxel(x, y + 1, z, true));
                            }
                        }
                        // If a voxel that contains the projection point and the current voxel are not the
                        // same, a distance from the voxel to the projection point is appended to the list.
                        else {
                            const double distance = Point<float>::distance(voxelPoint, projection);
                            distances.push_back(std::make_tuple(distance, projection, Point<float>((float)x, (float)y, (float)z)));  // Adding the tuple (distance, projection point, voxel point) to the list.
                        }
                    }
                }
            }

            // If there are no distances in the list, no pairs have to be checked. Yippee ki-yay!!!
            if (distances.empty()) {
                continue;
            }

            // Checking all pairs of before-calculated distances.
            for (unsigned long long i = 0; i < distances.size() - 1; i++) {
                for (unsigned long long j = i + 1; j < distances.size(); j++) {
                    // Getting the distances to the projection point and
                    // the projection points of the both tuples.
                    double distance1 = std::get<0>(distances[i]);
                    double distance2 = std::get<0>(distances[j]);
                    Point<float> projection1 = std::get<1>(distances[i]);
                    Point<float> projection2 = std::get<1>(distances[j]);

                    // If both distances and both projection points are the same (within the
                    // minimal tolerance), we add both interesting voxels to the symmetry.
                    if (Tolerance::isInTolerance(distance1, distance2, 0.0001) && projection1 == projection2) {
                        // Getting the voxel points.
                        Point<float> voxel1 = std::get<2>(distances[i]);
                        Point<float> voxel2 = std::get<2>(distances[j]);

                        // Adding the both voxels to the list.
                        symmetry.voxels.push_back(Voxel((int)voxel1.getX(), (int)voxel1.getY(), (int)voxel1.getZ(), true));
                        symmetry.voxels.push_back(Voxel((int)voxel2.getX(), (int)voxel2.getY(), (int)voxel2.getZ(), true));
                    }
                }
            }
        }
    }
}

// Removing small clusters of symmetries.
void ReflectionSymmetry::removeSmallClusters(std::vector<ReflectionSymmetry>& reflectionSymmetries, const int minimumClusterSize) {
    // If the minimum cluster size equals 1, no clusters have to be removed. Yaaayyy!
    if (minimumClusterSize == 1) {
        return;
    }

    // Removing clusters for each symmetry.
    for (ReflectionSymmetry& symmetry : reflectionSymmetries) {
        std::vector<std::set<Voxel>> clusters = symmetry.findClusters();  // Finding clusters for the current symmetry.

        // Removing clusters that contain too few voxels.
        for (unsigned long long i = 0; i < clusters.size(); i++) {
            const int clusterSize = (int)clusters[i].size();  // Reading the current cluster size (number of voxels).

            // If the current cluster size is smaller than the minimum cluster size, the cluster is removed from the symmetry.
            if (clusterSize < minimumClusterSize) {
                for (const Voxel& v : clusters[i]) {
                    std::vector<Voxel>::iterator position = std::find(symmetry.voxels.begin(), symmetry.voxels.end(), Voxel(v.getX(), v.getY(), v.getZ(), true, true));
                    symmetry.voxels.erase(position);
                }
            }
        }
    }

    // Removing empty symmetries.
    for (unsigned long long i = 0; i < reflectionSymmetries.size(); i++) {
        if (reflectionSymmetries[i].voxels.size() <= 0) {
            reflectionSymmetries.erase(reflectionSymmetries.begin() + i);
            i--;
        }
    }
}



// OBJECT METHODS
// Clustering recursive step.
void ReflectionSymmetry::clusterRecursiveStep(std::set<Voxel>& cluster, std::vector<std::vector<std::vector<Voxel>>>& voxelVector, const int x, const int y, const int z) const {
    // If the voxel coordinates lie outside of the voxel mesh or the
    // voxel has been already checked, the recursion unfolds.
    if (x < 0 || x >= voxelMesh.voxelX ||
        y < 0 || y >= voxelMesh.voxelY ||
        z < 0 || z >= voxelMesh.voxelZ ||
        voxelVector[z][y][x].getChecked()
    )
    {
        return;
    }

    // Setting the current voxel to checked.
    voxelVector[z][y][x].setChecked(true);

    // If the current voxel is not in symmetry, the recursion starts to unfold.
    if (!voxelVector[z][y][x].getInSymmetry()) {
        return;
    }

    // Inserting the current voxel to the cluster.
    cluster.insert(Voxel(x, y, z, true));

    // 26 recursive steps to for depth-first search of the neighborhood.
    // 9 in the upper layer, 8 (all but current) in the current layer, 9 in the lower layer.
    clusterRecursiveStep(cluster, voxelVector, x - 1, y - 1, z + 1);
    clusterRecursiveStep(cluster, voxelVector, x + 0, y - 1, z + 1);
    clusterRecursiveStep(cluster, voxelVector, x + 1, y - 1, z + 1);
    clusterRecursiveStep(cluster, voxelVector, x - 1, y + 0, z + 1);
    clusterRecursiveStep(cluster, voxelVector, x + 0, y + 0, z + 1);
    clusterRecursiveStep(cluster, voxelVector, x + 1, y + 0, z + 1);
    clusterRecursiveStep(cluster, voxelVector, x - 1, y + 1, z + 1);
    clusterRecursiveStep(cluster, voxelVector, x + 0, y + 1, z + 1);
    clusterRecursiveStep(cluster, voxelVector, x + 1, y + 1, z + 1);
    clusterRecursiveStep(cluster, voxelVector, x - 1, y - 1, z + 0);
    clusterRecursiveStep(cluster, voxelVector, x + 0, y - 1, z + 0);
    clusterRecursiveStep(cluster, voxelVector, x + 1, y - 1, z + 0);
    clusterRecursiveStep(cluster, voxelVector, x - 1, y + 0, z + 0);
    clusterRecursiveStep(cluster, voxelVector, x + 0, y + 0, z + 0);
    clusterRecursiveStep(cluster, voxelVector, x + 1, y + 0, z + 0);
    clusterRecursiveStep(cluster, voxelVector, x - 1, y + 1, z + 0);
    clusterRecursiveStep(cluster, voxelVector, x + 0, y + 1, z + 0);
    clusterRecursiveStep(cluster, voxelVector, x + 1, y + 1, z + 0);
    clusterRecursiveStep(cluster, voxelVector, x - 1, y - 1, z - 1);
    clusterRecursiveStep(cluster, voxelVector, x + 0, y - 1, z - 1);
    clusterRecursiveStep(cluster, voxelVector, x + 1, y - 1, z - 1);
    clusterRecursiveStep(cluster, voxelVector, x - 1, y + 0, z - 1);
    clusterRecursiveStep(cluster, voxelVector, x + 0, y + 0, z - 1);
    clusterRecursiveStep(cluster, voxelVector, x + 1, y + 0, z - 1);
    clusterRecursiveStep(cluster, voxelVector, x - 1, y + 1, z - 1);
    clusterRecursiveStep(cluster, voxelVector, x + 0, y + 1, z - 1);
    clusterRecursiveStep(cluster, voxelVector, x + 1, y + 1, z - 1);
}

// Finding clusters of symmetry voxels in symmetry.
std::vector<std::set<Voxel>> ReflectionSymmetry::findClusters() const {
    std::vector<std::set<Voxel>> clusters;

    // Getting the 3D voxel vector.
    auto voxels = getNormalisedVoxelVector();
    for (const Voxel& v : this->voxels) {
        // Setting all voxels in symmetry to true.
        voxels[v.getZ()][v.getY()][v.getX()].setInSymmetry(true);
    }

    // Searching for clusters in each voxel in symmetry (depth-first-search).
    for (const Voxel& v : this->voxels) {
        // Getting the coordinates of the voxel.
        const int x = v.getX();
        const int y = v.getY();
        const int z = v.getZ();

        // If the current voxel has not been checked and is in symmetry, a new cluster has been found.
        if (!voxels[z][y][x].getChecked() && voxels[z][y][x].getInSymmetry()) {
            std::set<Voxel> cluster;                         // Creating a new cluster.
            clusterRecursiveStep(cluster, voxels, x, y, z);  // Recursively search of the current cluster voxels.
            clusters.push_back(cluster);                     // Adding a new cluster to the list.
        }
    }

    return clusters;
}



// CONSTRUCTORS
// Default constructor.
ReflectionSymmetry::ReflectionSymmetry(
    Plane plane,
    std::vector<LineSegment> lineSegments
) :
    plane(plane),
    lineSegments(lineSegments)
{}

// Constructor with the plane and the voxels.
ReflectionSymmetry::ReflectionSymmetry(const Plane& plane, const std::vector<Voxel>& voxels) :
    plane(plane),
    voxels(voxels)
{}



// OVERLOADED OPERATORS
// Operator == is used for checking whether the two symmetries are equal.
bool ReflectionSymmetry::operator == (ReflectionSymmetry& s) {
    // First condition: same planes.
    if (plane == s.plane) {
        // Second condition: same voxel count.
        if (voxels.size() == s.voxels.size()) {
            // Third condition: same voxels.
            for (unsigned long long i = 0; i < voxels.size(); i++) {
                if (voxels[i] == s.voxels[i]) {
                    continue;
                }
                else {
                    return false;
                }
            }

            return true;
        }
    }

    return false;
}

// Operator &= is used for merging the current symmetry with another.
ReflectionSymmetry ReflectionSymmetry::operator &= (ReflectionSymmetry& s) {
    // Adding the line segments of the second symmetry.
    std::vector<LineSegment> lineSegments = s.getLineSegments();
    for (unsigned int i = 0; i < lineSegments.size(); i++) {
        this->addLineSegment(lineSegments[i]);
    }

    return* this;
}



// GETTERS AND SETTERS
// Symmetry plain vector getter.
const Plane ReflectionSymmetry::getPlane() const {
    return plane;
}

// Line segments getter.
const std::vector<LineSegment> ReflectionSymmetry::getLineSegments() const {
    return lineSegments;
}

// Getting number of line segments.
unsigned int ReflectionSymmetry::getLineSegmentCount() const {
    return lineSegments.size();
}

// Adding a new line segment.
void ReflectionSymmetry::addLineSegment(LineSegment lineSegment) {
    lineSegments.push_back(lineSegment);
}

// Voxel vector getter.
const std::vector<Voxel> ReflectionSymmetry::getVoxels() const {
    return voxels;
}



// CLASS METHODS
// Getting line segments between all pairs of points (fully connected graph).
std::vector<LineSegment> ReflectionSymmetry::calculateLineSegmentsBetweenPoints(
    std::vector<Point<float>> points,
    const double tolerance)
{
    std::vector<LineSegment> lineSegments;

    // Calculation of every pair of points.
    for (unsigned long long i = 0; i < points.size() - 1; i++) {
        for (unsigned long long j = i + 1; j < points.size(); j++) {
            // Cycles are not allowed in graph.
            // Line segments between points with greater Z coordinate
            // difference than tolerance are ignored.
            if (i != j && Tolerance::isInTolerance(points[i].getZ(), points[j].getZ(), tolerance)) {
                LineSegment ls(points[i], points[j]);
                double percent = ReflectionSymmetry::calculateInterestingPercentInLineSegment(ls);
                ls.setFullQuotient((float)percent);

                lineSegments.push_back(ls);
            }
        }
    }

    // Sorting all line segments by length.
    std::sort(
        lineSegments.begin(),
        lineSegments.end(),
        [](LineSegment a, LineSegment b) { return a.getLength() < b.getLength(); }
    );

    return lineSegments;
}

// Calculating reflection symmetries.
std::vector<ReflectionSymmetry> ReflectionSymmetry::calculateReflectionSymmetries(
    std::vector<LineSegment> lineSegments,
    const double tolerance,
    const int minimumClusterSize
)
{
    // Algorithm for finding reflection symmetries.
    std::vector<ReflectionSymmetry> symmetries = findSimpleSymmetries(lineSegments, tolerance);  // Finding simple (trivial) symmetries according to line segments.
    std::vector<ReflectionSymmetry> mergedSymmetries = mergeSymmetries(symmetries, tolerance);   // Merging the simple (trivial) symmetries with the given tolerance.
    getVoxelsInSymmetries(mergedSymmetries);                                                     // Adding voxels to each symmetry.
    addInterestingVoxelsToSymmetry(mergedSymmetries);                                            // Adding additional interesting voxels to each symmetry.
    removeSmallClusters(mergedSymmetries, minimumClusterSize);                                   // Removing voxels in symmetries that are in too small clusters.
    removeVoxelsWithoutPairs(mergedSymmetries);                                                  // Removing voxels that have no pairs across the symmetry plane.

    return mergedSymmetries;
}

// Finding partial symmetries from all reflection symmetries.
std::vector<ReflectionSymmetry> ReflectionSymmetry::findPartialSymmetries(const std::vector<ReflectionSymmetry>& reflectionSymmetries, const int minSymmetrySize) {
    std::vector<ReflectionSymmetry> partialSymmetries;

    // Iterating through all the reflection symmetries.
    for (const ReflectionSymmetry& symmetry : reflectionSymmetries) {
        std::vector<std::set<Voxel>> clusters = symmetry.findClusters();

        // Iterating through all the clusters.
        for (const std::set<Voxel>& cluster : clusters) {
            // If the cluster size is equal or bigger than the minimal
            // symmetry size, a new symmetry is added to the list.
            if (cluster.size() >= (unsigned long)minSymmetrySize) {
                const std::vector<Voxel> voxels(cluster.begin(), cluster.end());
                for (const Voxel& voxel : voxels) {
                    // Calculation of voxel and projection points.
                    const float voxelX = (voxel.getX() + 0.5f) * voxelMesh.voxelSideSize + voxelMesh.minX;
                    const float voxelY = (voxel.getY() + 0.5f) * voxelMesh.voxelSideSize + voxelMesh.minY;
                    const float voxelZ = (voxel.getZ() + 0.5f) * voxelMesh.voxelSideSize + voxelMesh.minZ;
                    const Point<float> voxelPoint(voxelX, voxelY, voxelZ);
                    const Point<float> projection = symmetry.plane.calculateProjectionPoint(voxelPoint, voxelMesh);

                    // Calculating the projection point voxel coordinates.
                    const int xp = (int)floor((projection.getX() - voxelMesh.minX) / voxelMesh.voxelSideSize);  // X index.
                    const int yp = (int)floor((projection.getY() - voxelMesh.minY) / voxelMesh.voxelSideSize);  // Y index.
                    const int zp = (int)floor((projection.getZ() - voxelMesh.minZ) / voxelMesh.voxelSideSize);  // Z index.

                    // If the voxel and the projection point voxel are the same,
                    // a reflection symmetry is also a partial symmetry.
                    if (voxel.getX() == xp && voxel.getY() == yp && voxel.getZ() && zp) {
                        partialSymmetries.push_back(ReflectionSymmetry(symmetry.plane, voxels));
                        break;
                    }
                    // If the symmetry plane goes through the edge of a voxel and is vertical,
                    // a reflection symmetry is also a partial symmetry.
                    else if (voxel.getX() == xp - 1 && !Tolerance::isInTolerance(xp * voxelMesh.voxelSideSize, projection.getX(), 0.0001) && symmetry.plane.getA() == 1) {
                        partialSymmetries.push_back(ReflectionSymmetry(symmetry.plane, voxels));
                        break;
                    }
                    // If the symmetry plane goes through the edge of a voxel and is horizontal,
                    // a reflection symmetry is also a partial symmetry.
                    else if (voxel.getX() == yp - 1 && !Tolerance::isInTolerance(yp * voxelMesh.voxelSideSize, projection.getY(), 0.0001) && symmetry.plane.getA() == 0) {
                        partialSymmetries.push_back(ReflectionSymmetry(symmetry.plane, voxels));
                        break;
                    }
                }
            }
        }
    }

    return partialSymmetries;
}



// OBJECT METHODS
// Calculating the point positions according to a plane.
std::vector<PositionFromPlane> ReflectionSymmetry::calculatePositionsFromPlane(const std::vector<Point<float>>& points, const Plane* plane) {
    const unsigned long long numberOfPoints = points.size();  // Getting the point vector size.

    // If there is no plane, all points are undefined.
    if (plane == nullptr) {
        return std::vector<PositionFromPlane>(numberOfPoints, PositionFromPlane::undefined);
    }

    // Creating a new vector with positions.
    std::vector<PositionFromPlane> positions(numberOfPoints, PositionFromPlane::undefined);
    for (unsigned long long i = 0; i < numberOfPoints; i++) {
        // Getting each single point and its voxel coordinates.
        const Point<float>& point = points[i];
        const int voxelX = (int)floor((point.getX() - voxelMesh.minX) / voxelMesh.voxelSideSize);
        const int voxelY = (int)floor((point.getY() - voxelMesh.minY) / voxelMesh.voxelSideSize);
        const int voxelZ = (int)floor((point.getZ() - voxelMesh.minZ) / voxelMesh.voxelSideSize);
        const int voxelXCoordinate = (int)(voxelX * voxelMesh.voxelSideSize + voxelMesh.minX);
        const int voxelYCoordinate = (int)(voxelY * voxelMesh.voxelSideSize + voxelMesh.minY);
        const int voxelZCoordinate = (int)(voxelZ * voxelMesh.voxelSideSize + voxelMesh.minZ);

        // If the current point lies outside of an interesting voxel, its position is set to undefined.
        if (std::find(voxels.begin(), voxels.end(), Voxel(voxelX, voxelY, voxelZ)) == voxels.end()) {
             positions[i] = PositionFromPlane::undefined;
             continue;
        }

        if (plane != nullptr) {
            // Calculating the center coordinates of the voxel where the point is located.
            const float voxelXCenter = (voxelX + 0.5f) * voxelMesh.voxelSideSize + voxelMesh.minX;
            const float voxelYCenter = (voxelY + 0.5f) * voxelMesh.voxelSideSize + voxelMesh.minY;
            const float voxelZCenter = (voxelZ + 0.5f) * voxelMesh.voxelSideSize + voxelMesh.minZ;
            const Point<float> voxelCenter(voxelXCenter, voxelYCenter, voxelZCenter);

            // Calculating the projection point to the symmetry plane.
            const Point<float> projection = plane->calculateProjectionPoint(voxelCenter, voxelMesh);
            const int voxelProjectionX = (int)floor((projection.getX() - voxelMesh.minX) / voxelMesh.voxelSideSize);
            const int voxelProjectionY = (int)floor((projection.getY() - voxelMesh.minY) / voxelMesh.voxelSideSize);
            const int voxelProjectionZ = (int)floor((projection.getZ() - voxelMesh.minZ) / voxelMesh.voxelSideSize);

            // If the projection point lies within the same voxel as the point, the position is set to center.
            // Note: voxels that are only touched by the symmetry plane on one edge are NOT center voxels.
            if (voxelX == voxelProjectionX && !Tolerance::isInTolerance(voxelXCoordinate, projection.getX(), 0.001) &&
                voxelY == voxelProjectionY && !Tolerance::isInTolerance(voxelYCoordinate, projection.getY(), 0.001) &&
                voxelZ == voxelProjectionZ && !Tolerance::isInTolerance(voxelZCoordinate, projection.getZ(), 0.001))
            {
                positions[i] = PositionFromPlane::center;
                continue;
            }

            // If the point is on the left side of the plane, left
            // position is added, right otherwise.
            if (plane->isPointOnTheLeftSide(point, voxelMesh)) {
                positions[i] = PositionFromPlane::left;
            }
            else {
                positions[i] = PositionFromPlane::right;
            }
        }
    }

    return positions;
}
