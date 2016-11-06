#ifndef BOX_H
#define BOX_H

#include "pointmass.h"
#include "spring.h"

class Box {
    PointMass mP[8];
    Spring mS[28];
    int mNumSprings = 28;
    float mK;

  public:
    Box();
    void draw();
    void simulate(float h);
    void applyGravity(float g, float h);
    void collideFloor(float level);
    PointMass* getClosestPoint(float minDist, Eigen::Vector3f p, Eigen::Vector3f dir);
};

#endif
