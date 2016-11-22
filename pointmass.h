#ifndef POINTMASS_H
#define POINTMASS_H

#include <Eigen/Dense>

class PointMass {
  public:
    Eigen::Vector3f mPos;
    Eigen::Vector3f mVel;
    float mMass;
    bool mImmobile;

    PointMass();
    PointMass(const PointMass& p);
    PointMass(float x, float y, float z);
    void simulate(float h);
    float calculateDepth(float minDist, Eigen::Vector3f p, Eigen::Vector3f dir);
    void draw();
};

#endif
