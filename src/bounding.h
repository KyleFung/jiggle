#ifndef BOUNDING_H
#define BOUNDING_H

#include <vector>

#include "interval.h"
#include "pointmass.h"
#include "triangle.h"
#include "edge.h"

class Bounding {
  public:
    Bounding(interval<PointMass>& points, interval<Triangle>& triangles, interval<Edge>& edges,
             float h);
    void refresh(float h);
    bool collide(Bounding b);
    void draw();
    Eigen::Vector3f getCentroid();

  private:
    // Representation of bounding sphere
    Eigen::Vector3f mCen;
    float mRad;
    float calculateBoundingRadius(float h);

    // Vector of integers indexing the vertices contained in this volume
    interval<PointMass>& mPointList;
    PointMass& getPoint(int i);

    // Vector of integers indexing the triangles contained in this volume
    interval<Triangle>& mTriangleList;
    Triangle& getTriangle(int i);

    // Vector of integers indexing the edge contained in this volume
    interval<Edge>& mEdgeList;
    Edge& getEdge(int i);
};

#endif
