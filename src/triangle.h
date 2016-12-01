#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "pointmass.h"

#include <Eigen/Dense>
#include <vector>

class Triangle {
  public:
    std::vector<PointMass>& mP;
    int mV[3];
    Triangle(std::vector<PointMass>& p, int x0, int x1, int x2);
    Eigen::Vector3f getPos(int i);
    Eigen::Vector3f getVel(int i);
    void draw();
    void simulate(float h);
    float collide(PointMass p, float h);
};

#endif
