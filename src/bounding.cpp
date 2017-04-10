#include "bounding.h"

#include <GLUT/glut.h>

Bounding::Bounding(Geometry& g, float h) : mG(g), depth(0), hasChildren(false) {
    for(int i = 0; i < childCount; i++) {
        mBC[i] = NULL;
        mGC[i] = g;
        mGC[i].setEmpty();
    }
    refresh(h);
}

Bounding::Bounding(Geometry& g, float h, int depth) : mG(g), depth(depth), hasChildren(false) {
    for(int i = 0; i < childCount; i++) {
        mBC[i] = NULL;
        mGC[i] = g;
        mGC[i].setEmpty();
    }
    refresh(h);
}

Bounding::~Bounding() {
    for(int i = 0; i < childCount; i++) {
        delete mBC[i];
    }
}

void Bounding::draw() {
    glTranslatef(mCen(0), mCen(1), mCen(2));
    glColor4f(1.0, 1.0, 1.0, 1.0);
    glutWireSphere(mRad, 10, 10);
    glTranslatef(-1 * mCen(0), -1 * mCen(1), -1 * mCen(2));

    // Recurse through the tree
    for(int i = 0; i < childCount; i++) {
        if(mBC[i])
            mBC[i]->draw();
    }
}

void Bounding::refresh(float h) {
    // If set of 0 geometry, then it is space of nothing
    if(mG.empty()) {
        mCen << 0, 0, 0;
        mRad = 0;
        return;
    }

    // Set cen to be the centroid of points, triangles, and edges
    mCen = getCentroid();

    // Set rad to be the farthest distance from cen
    mRad = calculateBoundingRadius(h);

    // Recurse through the tree
    for(int i = 0; i < childCount; i++) {
        if(mBC[i])
            mBC[i]->refresh(h);
    }
}

void Bounding::partition() {
    mCen = getCentroid();

    mG.partition(mCen, mGC);

    for(int i = 0; i < childCount; i++) {
        if(!mGC[i].empty()) {
            hasChildren = true;
            mBC[i] = new Bounding(mGC[i], 0, depth + 1);
            if(mGC[i].numberOfPrims() != mG.numberOfPrims())
                mBC[i]->partition();
        }
    }
}

float Bounding::calculateBoundingRadius(float h) {
    float radius = 0;
    int numPoints = mG.getPoints().size();
    for(int i = 0; i < numPoints; i++) {
        float distance = (getPoint(i).mPos - mCen).norm();
        if(distance > radius)
            radius = distance;
        distance = (getPoint(i).mPos + h * getPoint(i).mVel - mCen).norm();
        if(distance > radius)
            radius = distance;
    }
    int numTriangles = mG.getTriangles().size();
    for(int i = 0; i < numTriangles; i++) {
        Eigen::Vector3f furthest = getTriangle(i).getFurthest(mCen);
        Eigen::Vector3f fastest = getTriangle(i).getFastest();
        float distance = (furthest - mCen).norm() + h * fastest.norm();
        if(distance > radius)
            radius = distance;
    }
    int numEdges = mG.getEdges().size();
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
    int numPoints = mG.getPoints().size();
    for(int i = 0; i < numPoints; i++) {
        centroid += getPoint(i).mPos;
    }
    int numTriangles = mG.getTriangles().size();
    for(int i = 0; i < numTriangles; i++) {
        centroid += 3.0f * getTriangle(i).getCentroid();
    }
    int numEdges = mG.getEdges().size();
    for(int i = 0; i < numEdges; i++) {
        centroid += 2.0f * getEdge(i).getCentroid();
    }
    centroid /= (float) (numPoints + 3 * numTriangles + 2 * numEdges);
    return centroid;
}

PointMass& Bounding::getPoint(int i) {
    return mG.getPoints()[i];
}

Triangle& Bounding::getTriangle(int i) {
    return mG.getTriangles()[i];
}

Edge& Bounding::getEdge(int i) {
    return mG.getEdges()[i];
}

bool Bounding::isLeaf() {
    return !hasChildren;
}

bool Bounding::collide(Bounding& b, float h) {
    // Base case brute force
    if(isLeaf() && b.isLeaf()) {
        // Point face sweep 1
        int t0 = mG.getTriangles().size();
        int p1 = b.mG.getPoints().size();
        for(int i = 0; i < t0; i++) {
            for(int j = 0; j < p1; j++) {
                if(getTriangle(i).collide(b.getPoint(j), h) != -1) {
                    return true;
                }
            }
        }

        // Point face sweep 2
        int t1 = b.mG.getTriangles().size();
        int p0 = mG.getPoints().size();
        for(int i = 0; i < t1; i++) {
            for(int j = 0; j < p0; j++) {
                if(b.getTriangle(i).collide(getPoint(j), h) != -1) {
                    return true;
                }
            }
        }

        // Edge edge sweep
        int e0 = mG.getEdges().size();
        int e1 = b.mG.getEdges().size();
        for(int i = 0; i < e0; i++) {
            for(int j = 0; j < e1; j++) {
                if(getEdge(i).collide(b.getEdge(j), h) != -1) {
                    return true;
                }
            }
        }
        return false;
    }

    // Recursive check
    // Highest level
    float distance = (mCen - b.mCen).norm();
    if(distance > mRad + b.mRad) {
        return false;
    }

    // Next level
    for(int i = 0; i < childCount; i++) {
        if(mBC[i] && b.collide(*mBC[i], h)) {
            return true;
        }
    }

    return false;
}
