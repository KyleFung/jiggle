#include "geometry.h"

void Geometry::cleanUp() {
    mP.cleanUp();
    mT.cleanUp();
    mE.cleanUp();
}

void Geometry::setEmpty() {
    setEmptyP();
    setEmptyT();
    setEmptyE();
}

void Geometry::setEmptyP() {
    mP.setInterval(0, -1);
}

void Geometry::setEmptyT() {
    mT.setInterval(0, -1);
}

void Geometry::setEmptyE() {
    mE.setInterval(0, -1);
}

bool Geometry::empty() {
    return (mP.empty() && mT.empty() && mE.empty());
}

void Geometry::partition(Eigen::Vector3f cen, Geometry* children) {
    // Partition by x
    Geometry left(*this);
    Geometry right(*this);
    partition(cen, left, right, 0);

    // Partition by y
    Geometry topLeft(*this);
    Geometry btmLeft(*this);
    left.partition(cen, btmLeft, topLeft, 1);

    Geometry topRight(*this);
    Geometry btmRight(*this);
    right.partition(cen, btmRight, topRight, 1);

    // Partition by z
    for(int i = 0; i < 8; i++) {
        children[i] = Geometry(*this);
    }
    topLeft.partition(cen, children[0], children[1], 2);
    btmLeft.partition(cen, children[2], children[3], 2);
    topRight.partition(cen, children[4], children[5], 2);
    btmRight.partition(cen, children[6], children[7], 2);
}

void Geometry::partition(Eigen::Vector3f cen, Geometry& l, Geometry& r, int split) {
    // Partition about the split-th component
    // Points
    if(!mP.empty()) {
        std::vector<int>::iterator midP = std::partition(mP.getStart(), mP.getEnd(), isPointUnder(split, &mP, cen));
        if(mP.getStart() == midP)
            l.setEmptyP();
        else
            l.setIntervalP(mP.getStart(), midP - 1);
        if(midP == mP.getEnd() + 1)
            r.setEmptyP();
        else
            r.setIntervalP(midP, mP.getEnd());
    } else {
        l.setEmptyP();
        r.setEmptyP();
    }

    // Triangles
    if(!mT.empty()) {
        std::vector<int>::iterator midT = std::partition(mT.getStart(), mT.getEnd(), isTriangleUnder(split, &mT, cen));
        if(mT.getStart() == midT)
            l.setEmptyT();
        else
            l.setIntervalT(mT.getStart(), midT - 1);
        if(midT == mT.getEnd() + 1)
            r.setEmptyT();
        else
            r.setIntervalT(midT, mT.getEnd());
    } else {
        l.setEmptyT();
        r.setEmptyT();
    }

    // Edges
    if(!mE.empty()) {
        std::vector<int>::iterator midE = std::partition(mE.getStart(), mE.getEnd(), isEdgeUnder(split, &mE, cen));
        if(mE.getStart() == midE)
            l.setEmptyE();
        else
            l.setIntervalE(mE.getStart(), midE - 1);
        if(midE == mE.getEnd() + 1)
            r.setEmptyE();
        else
            r.setIntervalE(midE, mE.getEnd());
    }
    else {
        l.setEmptyE();
        r.setEmptyE();
    }
}

int Geometry::numberOfPrims() {
    return mP.size() + mT.size() + mE.size();
}

void Geometry::setIntervalP(std::vector<int>::iterator ps, std::vector<int>::iterator pe) {
    mP.setInterval(ps, pe);
}

void Geometry::setIntervalT(std::vector<int>::iterator ts, std::vector<int>::iterator te) {
    mT.setInterval(ts, te);
}

void Geometry::setIntervalE(std::vector<int>::iterator es, std::vector<int>::iterator ee) {
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