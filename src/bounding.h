#ifndef BOUNDING_H
#define BOUNDING_H

#include "pointmass.h"

class Bounding {
  public:
    Bounding();
    Bounding(PointMass* p, int n, float h);
    bool collide(Bounding b);

    Eigen::Vector3f mCen;
    float mRad;
};

#endif
