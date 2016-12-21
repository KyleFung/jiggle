#ifndef EDGE_H
#define EDGE_H

#include "interval.h"
#include "pointmass.h"

#include <vector>

class Edge {
  public:
    interval<PointMass>& mP;
    int mV[2];
    Edge(interval<PointMass>& p, int x0, int x1);
    Eigen::Vector3f getPos(int i);
    Eigen::Vector3f getVel(int i);
    void draw();
    void simulate(float h);
    float collide(Edge e, float h);

    Eigen::Vector3f getFastest();
    Eigen::Vector3f getFurthest(Eigen::Vector3f center);
    Eigen::Vector3f getCentroid();
};

#endif
