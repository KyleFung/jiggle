#include "geometry.h"

Interval<PointMass>* Geometry::sP = NULL;
Interval<Triangle>* Geometry::sT = NULL;
Interval<Edge>* Geometry::sE = NULL;
Eigen::Vector3f Geometry::sCen;

void Geometry::cleanUp() {
    mP.cleanUp();
    mT.cleanUp();
    mE.cleanUp();
}

void Geometry::setEmpty() {
    mP.setInterval(0, -1);
    mT.setInterval(0, -1);
    mE.setInterval(0, -1);
}

bool Geometry::empty() {
    return (mP.empty() && mT.empty() && mE.empty());
}

void Geometry::partition(Eigen::Vector3f cen, Geometry& l, Geometry& r) {
    setComparison(&mP, &mT, &mE, cen);
    std::vector<int>::iterator midP = std::partition(mP.getStart(), mP.getEnd(), isPointLeft);
    std::vector<int>::iterator midT = std::partition(mT.getStart(), mT.getEnd(), isTriangleLeft);
    std::vector<int>::iterator midE = std::partition(mE.getStart(), mE.getEnd(), isEdgeLeft);

    l.setIntervals(mP.getStart(), midP - 1, mT.getStart(), midT - 1, mE.getStart(), midE - 1);
    r.setIntervals(midP, mP.getEnd(), midT, mT.getEnd(), midE, mE.getEnd());
}

void Geometry::setIntervals(std::vector<int>::iterator ps, std::vector<int>::iterator pe,
                            std::vector<int>::iterator ts, std::vector<int>::iterator te,
                            std::vector<int>::iterator es, std::vector<int>::iterator ee) {
    mP.setInterval(ps, pe);
    mT.setInterval(ts, te);
    mE.setInterval(es, ee);
}

Interval<PointMass>& Geometry::getPoints() {
    return mP;
}

Interval<Triangle>& Geometry::getTriangles() {
    return mT;
}

Interval<Edge>& Geometry::getEdges() {
    return mE;
}

void Geometry::addPoint(PointMass& p) {
    mP.addItem(p);
}

void Geometry::addTriangle(Triangle& t) {
    mT.addItem(t);
}

void Geometry::addEdge(Edge& e) {
    mE.addItem(e);
}

void Geometry::setComparison(Interval<PointMass>* p, Interval<Triangle>* t,
    Interval<Edge>* e, Eigen::Vector3f cen) {
    sP = p;
    sT = t;
    sE = e;
    sCen = cen;
}

bool Geometry::isPointLeft(int i) {
    return sP->getItem(i).mPos[0] < sCen[0];
}

bool Geometry::isTriangleLeft(int i) {
    for (int j = 0; j < 3; j++) {
        if (sT->getItem(i).getPos(j)[0] < sCen[0])
            return true;
    }
    return false;
}

bool Geometry::isEdgeLeft(int i) {
    for (int j = 0; j < 2; j++) {
        if (sE->getItem(i).getPos(j)[0] < sCen[0])
            return true;
    }
    return false;
}