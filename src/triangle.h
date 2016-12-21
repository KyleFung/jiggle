#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "interval.h"
#include "pointmass.h"

#include <Eigen/Dense>
#include <vector>

class Triangle {
  public:
    interval<PointMass>& mP;
    int mV[3];
    Triangle(interval<PointMass>& p, int x0, int x1, int x2);
    Eigen::Vector3f getPos(int i);
    Eigen::Vector3f getVel(int i);
    void draw();
    void simulate(float h);
    float collide(PointMass p, float h);

    Eigen::Vector3f getFastest();
    Eigen::Vector3f getFurthest(Eigen::Vector3f center);
    Eigen::Vector3f getCentroid();
};

#endif
