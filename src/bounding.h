#ifndef BOUNDING_H
#define BOUNDING_H

#include <vector>

#include "geometry.h"
#include "interval.h"
#include "pointmass.h"
#include "triangle.h"
#include "edge.h"

class Bounding {
  public:
    Bounding(Geometry& geometry, float h);
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

    // Children in bounding tree
    Bounding* mLeft;
    Bounding* mRight;
    Geometry mGL, mGR;

    // Vector of integers indexing the vertices contained in this volume
    Geometry& mG;
    PointMass& getPoint(int i);
    Triangle& getTriangle(int i);
    Edge& getEdge(int i);
};

#endif
