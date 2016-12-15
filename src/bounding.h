#ifndef BOUNDING_H
#define BOUNDING_H

#include <vector>

#include "pointmass.h"
#include "triangle.h"
#include "edge.h"

class Bounding {
  public:
    Bounding(std::vector<PointMass>& points, std::vector<int>& pindices,
             std::vector<Triangle>& triangles, std::vector<int>& tindices,
             std::vector<Edge>& edges, std::vector<int>& eindices,
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
    std::vector<PointMass>& mPointList;
    std::vector<int>& mPIndices;
    PointMass& getPoint(int i);

    // Vector of integers indexing the triangles contained in this volume
    std::vector<Triangle>& mTriangleList;
    std::vector<int>& mTIndices;
    Triangle& getTriangle(int i);

    // Vector of integers indexing the edge contained in this volume
    std::vector<Edge>& mEdgeList;
    std::vector<int>& mEIndices;
    Edge& getEdge(int i);
};

#endif
