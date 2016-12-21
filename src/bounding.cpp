#include "bounding.h"

#include <GLUT/glut.h>

Bounding::Bounding(interval<PointMass>& points, interval<Triangle>& triangles, interval<Edge>& edges,
    float h) : mPointList(points), mTriangleList(triangles), mEdgeList(edges) {
    refresh(h);
}

void Bounding::draw() {
    glTranslatef(mCen(0), mCen(1), mCen(2));
    glColor4f(1.0, 1.0, 1.0, 1.0);
    glutWireSphere(mRad, 10, 10);
    glTranslatef(-1 * mCen(0), -1 * mCen(1), -1 * mCen(2));
}

void Bounding::refresh(float h) {
    // If set of 0 geometry, then it is space of nothing
    if (mPointList.empty() && mTriangleList.empty() && mEdgeList.empty()) {
        mCen << 0, 0, 0;
        mRad = 0;
        return;
    }

    // Set cen to be the centroid of points, triangles, and edges
    mCen = getCentroid();

    // Set rad to be the farthest distance from cen
    mRad = calculateBoundingRadius(h);
}

float Bounding::calculateBoundingRadius(float h) {
    float radius = 0;
    int numPoints = mPointList.size();
    for(int i = 0; i < numPoints; i++) {
        float distance = (getPoint(i).mPos - mCen).norm();
        if(distance > radius)
            radius = distance;
        distance = (getPoint(i).mPos + h * getPoint(i).mVel - mCen).norm();
        if(distance > radius)
            radius = distance;
    }
    int numTriangles = mTriangleList.size();
    for(int i = 0; i < numTriangles; i++) {
        Eigen::Vector3f furthest = getTriangle(i).getFurthest(mCen);
        Eigen::Vector3f fastest = getTriangle(i).getFastest();
        float distance = (furthest - mCen).norm() + h * fastest.norm();
        if(distance > radius)
            radius = distance;
    }
    int numEdges = mEdgeList.size();
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
    int numPoints = mPointList.size();
    for(int i = 0; i < numPoints; i++) {
        centroid += getPoint(i).mPos;
    }
    int numTriangles = mTriangleList.size();
    for(int i = 0; i < numTriangles; i++) {
        centroid += 3.0f * getTriangle(i).getCentroid();
    }
    int numEdges = mEdgeList.size();
    for(int i = 0; i < numEdges; i++) {
        centroid += 2.0f * getEdge(i).getCentroid();
    }
    centroid /= (float) (numPoints + 3 * numTriangles + 2 * numEdges);
    return centroid;
}

PointMass& Bounding::getPoint(int i) {
    return mPointList[i];
}

Triangle& Bounding::getTriangle(int i) {
    return mTriangleList[i];
}

Edge& Bounding::getEdge(int i) {
    return mEdgeList[i];
}

bool Bounding::collide(Bounding b) {
    float distance = (mCen - b.mCen).norm();
    if(distance > mRad + b.mRad) {
        return false;
    }
    return true;
}
