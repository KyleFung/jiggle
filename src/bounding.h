#ifndef BOUNDING_H
#define BOUNDING_H

#include <vector>

#include "geometry.h"
#include "interval.h"
#include "pointmass.h"
#include "triangle.h"
#include "edge.h"

const int childCount = 8;

class Bounding {
  public:
    Bounding(Geometry& geometry, float h);
    Bounding(Geometry& geometry, float h, int depth);
    ~Bounding();
    void refresh(float h);
    void partition();
    bool collide(Bounding& b);
    void draw();
    Eigen::Vector3f getCentroid();

  private:
    // Representation of bounding sphere
    Eigen::Vector3f mCen;
    float mRad;
    float calculateBoundingRadius(float h);
    int depth;

    // Children in bounding tree
    Bounding* mBC[childCount];
    Geometry mGC[childCount];

    // Vector of integers indexing the vertices contained in this volume
    Geometry& mG;
    PointMass& getPoint(int i);
    Triangle& getTriangle(int i);
    Edge& getEdge(int i);
};

#endif
