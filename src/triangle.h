#ifndef TRIANGLE_H
#define TRIANGLE_H

#include "collision.h"
#include "interval.h"
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
    Collision collide(PointMass p, float h);

    Eigen::Vector3f getFastest();
    Eigen::Vector3f getFurthest(Eigen::Vector3f center);
    Eigen::Vector3f getCentroid();
    Eigen::Vector3f getNormal();

    bool isCoplanar(PointMass p);
};

#endif
