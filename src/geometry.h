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
    bool empty();

    void partition(Eigen::Vector3f cen, Geometry& l, Geometry& r);
    void setIntervals(std::vector<int>::iterator ps, std::vector<int>::iterator pe,
                      std::vector<int>::iterator ts, std::vector<int>::iterator te,
                      std::vector<int>::iterator es, std::vector<int>::iterator ee);

    Interval<PointMass>& getPoints();
    Interval<Triangle>& getTriangles();
    Interval<Edge>& getEdges();
    void addPoint(PointMass& p);
    void addTriangle(Triangle& t);
    void addEdge(Edge& e);

  private:
    // Partitioning functions and comparison assets
    static bool isPointLeft(int i);
    static bool isTriangleLeft(int i);
    static bool isEdgeLeft(int i);
    static void setComparison(Interval<PointMass>* p, Interval<Triangle>* t,
                              Interval<Edge>* e, Eigen::Vector3f cen);
    static Interval<PointMass>* sP;
    static Interval<Triangle>* sT;
    static Interval<Edge>* sE;
    static Eigen::Vector3f sCen;
};

#endif