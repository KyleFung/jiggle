#ifndef MESH_H
#define MESH_H

#include <vector>

#include "bounding.h"
#include "edge.h"
#include "pointmass.h"
#include "spring.h"
#include "triangle.h"

class Mesh {
  protected:
    // Mesh is represented by a list of point masses and springs
    // Another list of triangles and edges is used to represent its surface
    std::vector<PointMass> mP;
    std::vector<Spring> mS;
    std::vector<Triangle> mT;
    std::vector<Edge> mE;

    // Bounding volume for collision acceleration
    Bounding mB;

    float mK;

  public:
    // Accessors
    Mesh();
    void addPoint(PointMass p);
    void addSpring(Spring s);
    void addTriangle(Triangle t);
    void addEdge(Edge t);

    PointMass* getPoint(int i);
    Spring* getSpring(int i);
    Triangle* getTriangle(int i);
    Edge* getEdge(int i);

    // Collision routines
    Bounding getBounding();
    bool collide(Mesh m, float h);
    void refreshBounding(float h);

    void translate(Eigen::Vector3f pos);
    void simulate(float h);
    void applyGravity(float g, float h);
    void collideFloor(float level);
    PointMass* getClosestPoint(float minDist, Eigen::Vector3f p, Eigen::Vector3f dir);

    virtual ~Mesh();
    virtual void draw();
};

#endif
