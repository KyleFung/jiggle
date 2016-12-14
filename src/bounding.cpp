#include "bounding.h"

#include <GLUT/glut.h>

Bounding::Bounding(std::vector<PointMass>& points, std::vector<int>& indices, float h) :
mPointList(points), mIndices(indices) {
    refresh(h);
}

void Bounding::draw() {
    glTranslatef(mCen(0), mCen(1), mCen(2));
    glColor4f(1.0, 1.0, 1.0, 1.0);
    glutWireSphere(mRad, 10, 10);
    glTranslatef(-1 * mCen(0), -1 * mCen(1), -1 * mCen(2));
}

void Bounding::refresh(float h) {
    // If set of 0 points, then it is space of nothing
    if (mIndices.empty()) {
        mCen << 0, 0, 0;
        mRad = 0;
        return;
    }

    // Set cen to be the centroid of p
    mCen << 0, 0, 0;
    int numPoints = mIndices.size();
    for (int i = 0; i < numPoints; i++) {
        mCen += getPoint(i).mPos;
    }
    mCen /= (float) numPoints;

    // Set rad to be the farthest distance from cen
    mRad = 0;
    for (int i = 0; i < numPoints; i++) {
        float distance = (getPoint(i).mPos - mCen).norm();
        if (distance > mRad)
            mRad = distance;
        distance = (getPoint(i).mPos + h * getPoint(i).mVel - mCen).norm();
        if (distance > mRad)
            mRad = distance;
    }
}

PointMass& Bounding::getPoint(int i) {
    return mPointList[mIndices[i]];
}

bool Bounding::collide(Bounding b) {
    float distance = (mCen - b.mCen).norm();
    if(distance > mRad + b.mRad) {
        return false;
    }
    return true;
}
