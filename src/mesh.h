#ifndef MESH_H
#define MESH_H

#include <vector>

#include "bounding.h"
#include "edge.h"
#include "interval.h"
#include "pointmass.h"
#include "spring.h"
#include "triangle.h"

class Mesh {
  private:
    // Geometry expressed as a list of points, triangles, and edges
    interval<PointMass> mP;
    interval<Triangle> mT;
    interval<Edge> mE;
    Bounding mB;

    // Physical structure expressed as a list of springs
    std::vector<Spring> mS;

  public:
    Mesh();
    void addPoint(PointMass p);
    void addSpring(Spring s);
    void addTriangle(Triangle t);
    void addEdge(Edge t);

    float mK;

    interval<PointMass>& getPointList();

    // Collision routines
    Bounding& getBounding();
    bool collide(Mesh& m, float h);
    void partitionBounding();
    void refreshBounding(float h);
    void drawBounding();

    void translate(Eigen::Vector3f pos);
    void simulate(float h);
    void applyGravity(float g, float h);
    void collideFloor(float level);
    PointMass* getClosestPoint(float minDist, Eigen::Vector3f p, Eigen::Vector3f dir);

    virtual ~Mesh();
    virtual void draw();
};

#endif
