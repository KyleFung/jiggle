#include "bounding.h"

#include <GLUT/glut.h>

Interval<PointMass>* Bounding::sP = NULL;
Interval<Triangle>* Bounding::sT = NULL;
Interval<Edge>* Bounding::sE = NULL;
Eigen::Vector3f Bounding::sCen;

Bounding::Bounding(Interval<PointMass>& points, Interval<Triangle>& triangles, Interval<Edge>& edges,
    float h) : mP(points), mT(triangles), mE(edges),
    mPL(points.getBaseList(), points.getIndexList()), mPR(points.getBaseList(), points.getIndexList()),
    mTL(triangles.getBaseList(), triangles.getIndexList()), mTR(triangles.getBaseList(), triangles.getIndexList()),
    mEL(edges.getBaseList(), edges.getIndexList()), mER(edges.getBaseList(), edges.getIndexList()),
    mLeft(NULL), mRight(NULL) {
    refresh(h);
}

Bounding::~Bounding() {
    delete mLeft;
    delete mRight;
}

void Bounding::draw() {
    glTranslatef(mCen(0), mCen(1), mCen(2));
    glColor4f(1.0, 1.0, 1.0, 1.0);
    glutWireSphere(mRad, 10, 10);
    glTranslatef(-1 * mCen(0), -1 * mCen(1), -1 * mCen(2));

    // Recurse through the tree
    if(mLeft)
        mLeft->draw();
    if(mRight)
        mRight->draw();
}

void Bounding::refresh(float h) {
    // If set of 0 geometry, then it is space of nothing
    if (mP.empty() && mT.empty() && mE.empty()) {
        mCen << 0, 0, 0;
        mRad = 0;
        return;
    }

    // Set cen to be the centroid of points, triangles, and edges
    mCen = getCentroid();

    // Set rad to be the farthest distance from cen
    mRad = calculateBoundingRadius(h);

    // Recurse through the tree
    if(mLeft)
        mLeft->refresh(h);
    if (mRight)
        mRight->refresh(h);
}

void Bounding::partition() {
    mCen = getCentroid();
    setComparison(&mP, &mT, &mE, mCen);
    std::vector<int>::iterator midP = std::partition(mP.getStart(), mP.getEnd(), isPointLeft);
    std::vector<int>::iterator midT = std::partition(mT.getStart(), mT.getEnd(), isTriangleLeft);
    std::vector<int>::iterator midE = std::partition(mE.getStart(), mE.getEnd(), isEdgeLeft);

    mPL.setInterval(mP.getStart(), midP - 1);
    mPR.setInterval(midP, mP.getEnd());
    mTL.setInterval(mT.getStart(), midT - 1);
    mTR.setInterval(midT, mT.getEnd());
    mEL.setInterval(mE.getStart(), midE - 1);
    mER.setInterval(midE, mE.getEnd());

    mLeft = new Bounding(mPL, mTL, mEL, 0);
    mRight = new Bounding(mPR, mTR, mER, 0);
}

void Bounding::setComparison(Interval<PointMass>* points, Interval<Triangle>* triangles,
                             Interval<Edge>* edges, Eigen::Vector3f cen) {
    sP = points;
    sT = triangles;
    sE = edges;
    sCen = cen;
}

bool Bounding::isPointLeft(int i) {
    return sP->getItem(i).mPos[0] < sCen[0];
}

bool Bounding::isTriangleLeft(int i) {
    for(int j = 0; j < 3; j++) {
        if(sT->getItem(i).getPos(j)[0] < sCen[0])
            return true;
    }
    return false;
}

bool Bounding::isEdgeLeft(int i) {
    for (int j = 0; j < 2; j++) {
        if (sE->getItem(i).getPos(j)[0] < sCen[0])
            return true;
    }
    return false;
}

float Bounding::calculateBoundingRadius(float h) {
    float radius = 0;
    int numPoints = mP.size();
    for(int i = 0; i < numPoints; i++) {
        float distance = (getPoint(i).mPos - mCen).norm();
        if(distance > radius)
            radius = distance;
        distance = (getPoint(i).mPos + h * getPoint(i).mVel - mCen).norm();
        if(distance > radius)
            radius = distance;
    }
    int numTriangles = mT.size();
    for(int i = 0; i < numTriangles; i++) {
        Eigen::Vector3f furthest = getTriangle(i).getFurthest(mCen);
        Eigen::Vector3f fastest = getTriangle(i).getFastest();
        float distance = (furthest - mCen).norm() + h * fastest.norm();
        if(distance > radius)
            radius = distance;
    }
    int numEdges = mE.size();
    for(int i = 0; i < numEdges; i++) {
        Eigen::Vector3f furthest = getEdge(i).getFurthest(mCen);
        Eigen::Vector3f fastest = getEdge(i).getFastest();
        float distance = (furthest - mCen).norm() + h * fastest.norm();
        if(distance > radius)
            radius = distance;
    }
    return radius;
}

Eigen::Vector3f Bounding::getCentroid() {
    Eigen::Vector3f centroid;
    centroid << 0, 0, 0;
    int numPoints = mP.size();
    for(int i = 0; i < numPoints; i++) {
        centroid += getPoint(i).mPos;
    }
    int numTriangles = mT.size();
    for(int i = 0; i < numTriangles; i++) {
        centroid += 3.0f * getTriangle(i).getCentroid();
    }
    int numEdges = mE.size();
    for(int i = 0; i < numEdges; i++) {
        centroid += 2.0f * getEdge(i).getCentroid();
    }
    centroid /= (float) (numPoints + 3 * numTriangles + 2 * numEdges);
    return centroid;
}

PointMass& Bounding::getPoint(int i) {
    return mP[i];
}

Triangle& Bounding::getTriangle(int i) {
    return mT[i];
}

Edge& Bounding::getEdge(int i) {
    return mE[i];
}

bool Bounding::collide(Bounding& b) {
    float distance = (mCen - b.mCen).norm();
    if(distance > mRad + b.mRad) {
        return false;
    }
    return true;
}
