#ifndef EDGE_H
#define EDGE_H

#include "pointmass.h"

#include <vector>

class Edge {
  public:
    std::vector<PointMass>& mP;
    int mV[2];
    Edge(std::vector<PointMass>& p, int x0, int x1);
    Eigen::Vector3f getPos(int i);
    Eigen::Vector3f getVel(int i);
    void draw();
    void simulate(float h);
    float collide(Edge e, float h);
};

#endif
