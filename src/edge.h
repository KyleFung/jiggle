#ifndef EDGE_H
#define EDGE_H

#include "interval.h"
#include "pointmass.h"
#include "collision.h"

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
    Collision collide(Edge e, float h);

    Eigen::Vector3f getFastest();
    Eigen::Vector3f getFurthest(Eigen::Vector3f center);
    Eigen::Vector3f getCentroid();
    Eigen::Vector3f getNormal(Edge& e);

    bool isCoplanar(Edge other);
};

#endif
