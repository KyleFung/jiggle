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
    interval<PointMass> mPL, mPR;
    interval<Triangle> mTL, mTR;
    interval<Edge> mEL, mER;

    // Partitioning functions and comparison assets
    static bool isPointLeft(int i);
    static bool isTriangleLeft(int i);
    static bool isEdgeLeft(int i);
    static void setComparison(interval<PointMass>* points, interval<Triangle>* triangles,
                              interval<Edge>* edges, Eigen::Vector3f cen);
    static interval<PointMass>* sP;
    static interval<Triangle>* sT;
    static interval<Edge>* sE;
    static Eigen::Vector3f sCen;

    // Vector of integers indexing the vertices contained in this volume
    interval<PointMass>& mP;
    PointMass& getPoint(int i);

    // Vector of integers indexing the triangles contained in this volume
    interval<Triangle>& mT;
    Triangle& getTriangle(int i);

    // Vector of integers indexing the edge contained in this volume
    interval<Edge>& mE;
    Edge& getEdge(int i);
};

#endif
