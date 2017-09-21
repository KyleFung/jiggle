#ifndef MESH_H
#define MESH_H

#include <vector>

#include "bounding.h"
#include "edge.h"
#include "geometry.h"
#include "interval.h"
#include "pointmass.h"
#include "spring.h"
#include "triangle.h"
#include "collision.h"
#include "utils.h"

class Mesh {
  private:
    // Physical structure expressed as a list of springs
    std::vector<Spring> mS;
    Geometry mG;
    Bounding mB;
    Edge& getEdge(int i);
    PointMass& getPoint(int i);
    Triangle& getTriangle(int i);

  public:
    Mesh(std::string name);
    Geometry& getGeometry();
    void addPoint(PointMass p);
    void addSpring(Spring s);
    void addTriangle(Triangle t);
    void addEdge(Edge t);

    float mK;
    std::string mName;

    Interval<PointMass>& getPointList();

    // Collision routines
    Bounding& getBounding();
    Collision collide(Mesh& m, float h);
    void partitionBounding();
    void refreshBounding(float h);
    void drawBounding();

    void translate(Eigen::Vector3f pos);
    void rotate(Eigen::Vector3f axis, float degrees);
    void simulate(float h);
    void setName(std::string name);
    void applyGravity(float g, float h);
    void collideFloor(float level);
    PointMass* getClosestPoint(float minDist, Eigen::Vector3f p, Eigen::Vector3f dir);

    virtual ~Mesh();
    virtual void draw();
};

#endif
