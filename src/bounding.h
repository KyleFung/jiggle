#ifndef BOUNDING_H
#define BOUNDING_H

#include "pointmass.h"

#include <vector>

class Bounding {
  public:
    Bounding(std::vector<PointMass>& points, std::vector<int>& indices, float h);
    void refresh(float h);
    bool collide(Bounding b);
    void draw();

  private:
    // Representation of bounding sphere
    Eigen::Vector3f mCen;
    float mRad;

    // Vector of integers indexing the vertices contained in this volume
    std::vector<PointMass>& mPointList;
    std::vector<int>& mIndices;
    PointMass& getPoint(int i);
};

#endif
