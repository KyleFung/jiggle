#ifndef MESH_H
#define MESH_H

#include <vector>

#include "pointmass.h"
#include "spring.h"
#include "triangle.h"

class Mesh {
  protected:
    // Mesh is represented by a list of point masses and springs
    // Another list of triangles is used to represent its surface
    std::vector<PointMass> mP;
    std::vector<Spring> mS;
    std::vector<Triangle> mT;

    float mK;

  public:
    Mesh();
    void addPoint(PointMass p);
    void addSpring(Spring s);
    void addTriangle(Triangle t);

    PointMass* getPoint(int i);
    Spring* getSpring(int i);
    Triangle* getTriangle(int i);

    void simulate(float h);
    void applyGravity(float g, float h);
    void collideFloor(float level);
    PointMass* getClosestPoint(float minDist, Eigen::Vector3f p, Eigen::Vector3f dir);

    virtual ~Mesh();
    virtual void draw();
};

#endif
