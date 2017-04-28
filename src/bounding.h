#ifndef BOUNDING_H
#define BOUNDING_H

#define OPT_BVH 1

#include <vector>

#include "geometry.h"
#include "interval.h"
#include "pointmass.h"
#include "triangle.h"
#include "edge.h"
#include "collision.h"

#if OPT_BVH == 1
const int childCount = 8;
#endif
#if OPT_BVH == 0
const int childCount = 0;
#endif

class Bounding {
  public:
    Bounding(Geometry& geometry, float h);
    Bounding(Geometry& geometry, float h, int depth);
    ~Bounding();
    void refresh(float h);
    void partition();
    Collision collide(Bounding& b, float h);
    void draw();
    Eigen::Vector3f getCentroid();
    bool isLeaf();

  private:
    // Representation of bounding sphere
    Eigen::Vector3f mCen;
    float mRad;
    float calculateBoundingRadius(float h);
    int depth;
    bool hasChildren;

    // Children in bounding tree
#if OPT_BVH == 1
    Bounding* mBC[childCount];
    Geometry mGC[childCount];
#endif
#if OPT_BVH == 0
    Bounding* mBC[1];
    Geometry mGC[1];
#endif

    // Vector of integers indexing the vertices contained in this volume
    Geometry& mG;
    PointMass& getPoint(int i);
    Triangle& getTriangle(int i);
    Edge& getEdge(int i);
    int getPointBaseIndex(int i);
    int getTriangleBaseIndex(int i);
    int getEdgeBaseIndex(int i);
};

#endif
