#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "edge.h"
#include "interval.h"
#include "pointmass.h"
#include "triangle.h"

class Geometry
{
  private:
    Interval<PointMass> mP;
    Interval<Triangle> mT;
    Interval<Edge> mE;

  public:
    void cleanUp();
    void setEmpty();
    void setEmptyP();
    void setEmptyT();
    void setEmptyE();
    void setIntervalP(std::vector<int>::iterator ps, std::vector<int>::iterator pe);
    void setIntervalT(std::vector<int>::iterator ts, std::vector<int>::iterator te);
    void setIntervalE(std::vector<int>::iterator es, std::vector<int>::iterator ee);
    bool empty();
    int numberOfPrims();

    void partition(Eigen::Vector3f cen, Geometry* children);
    void partition(Eigen::Vector3f cen, Geometry& l, Geometry& r, int split);

    Interval<PointMass>& getPoints();
    Interval<Triangle>& getTriangles();
    Interval<Edge>& getEdges();
    void addPoint(PointMass& p);
    void addTriangle(Triangle& t);
    void addEdge(Edge& e);
};

class isPointUnder {
    int dim;
    Interval<PointMass>* points;
    Eigen::Vector3f cen;

public:
    isPointUnder(int d, Interval<PointMass>* p, Eigen::Vector3f c) : dim(d), points(p), cen(c) {}

    bool operator()(int i) const {
        return points->getItemFromBase(i).mPos[dim] < cen[dim];
    }
};

class isEdgeUnder {
    int dim;
    Interval<Edge>* edges;
    Eigen::Vector3f cen;

public:
    isEdgeUnder(int d, Interval<Edge>* e, Eigen::Vector3f c) : dim(d), edges(e), cen(c) {}

    bool operator()(int i) const {
        for(int j = 0; j < 2; j++) {
            if(edges->getItemFromBase(i).getPos(j)[dim] < cen[dim])
                return true;
        }
        return false;
    }
};

class isTriangleUnder {
    int dim;
    Interval<Triangle>* triangles;
    Eigen::Vector3f cen;

public:
    isTriangleUnder(int d, Interval<Triangle>* t, Eigen::Vector3f c) : dim(d), triangles(t), cen(c) {}

    bool operator()(int i) const {
        for(int j = 0; j < 3; j++) {
            if(triangles->getItemFromBase(i).getPos(j)[dim] < cen[dim])
                return true;
        }
        return false;
    }
};

#endif