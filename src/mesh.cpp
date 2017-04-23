#include "mesh.h"

#include <GLUT/glut.h>

#include <math.h>

#include "interval.h"

Mesh::Mesh() : mB(mG, 0) {}

Mesh::~Mesh() {
    mG.cleanUp();
}

void Mesh::draw() {
    int numTriangles = mG.getTriangles().size();
    for(int i = 0; i < numTriangles; i++) {
        getTriangle(i).draw();
    }
}

void Mesh::translate(Eigen::Vector3f pos) {
    int numNodes = mG.getPoints().size();
    for(int i = 0; i < numNodes; i++) {
        getPoint(i).mPos += pos;
    }
}

void Mesh::applyGravity(float g, float h) {
    int numNodes = mG.getPoints().size();
    for(int i = 0; i < numNodes; i++) {
        getPoint(i).mVel(1) -= (g * h) / getPoint(i).mMass;
    }
}

void Mesh::collideFloor(float level) {
    int numNodes = mG.getPoints().size();
    for(int i = 0; i < numNodes; i++) {
        if (getPoint(i).mPos(1) <= level) {
            getPoint(i).mPos(1) = level + 0.001;
            getPoint(i).mVel(1) = std::abs(getPoint(i).mVel(1));
            getPoint(i).mVel *= 0.9;
        }
    }
}

void Mesh::simulate(float h) {
    int numSprings = mS.size();
    for(int i = 0; i < numSprings; i++) {
        mS[i].contributeImpulse(h);
    }
    int numNodes = mG.getPoints().size();
    for(int i = 0; i < numNodes; i++) {
        getPoint(i).simulate(h);
    }
    refreshBounding(h);
}

PointMass* Mesh::getClosestPoint(float minDist, Eigen::Vector3f p, Eigen::Vector3f dir) {
    PointMass* closest = NULL;
    float closestDepth = 10000;

    int numNodes = mG.getPoints().size();
    for(int i = 0; i < numNodes; i++) {
        float depth = getPoint(i).calculateDepth(minDist, p, dir);
        // If the point is not behind the camera, and closer than the previous point
        if(depth > 0 && depth < closestDepth) {
            closest = &getPoint(i);
            closestDepth = depth;
        }
    }
    return closest;
}

bool Mesh::collide(Mesh& m, float h) {
    // Hierarchical scheme
    return mB.collide(m.getBounding(), h).exists();
}

Bounding& Mesh::getBounding() {
    return mB;
}

void Mesh::partitionBounding() {
    mB.partition();
}

void Mesh::refreshBounding(float h) {
    mB.refresh(h);
}

void Mesh::drawBounding() {
    mB.draw();
}

void Mesh::addPoint(PointMass p) {
    mG.addPoint(p);
}

void Mesh::addSpring(Spring s) {
    mS.push_back(s);
}

void Mesh::addTriangle(Triangle t) {
    mG.addTriangle(t);
}

void Mesh::addEdge(Edge e) {
    mG.addEdge(e);
}

Edge& Mesh::getEdge(int i) {
    return mG.getEdges().getBaseList()->at(i);
}

PointMass& Mesh::getPoint(int i) {
    return mG.getPoints().getBaseList()->at(i);
}

Triangle& Mesh::getTriangle(int i) {
    return mG.getTriangles().getBaseList()->at(i);
}

Interval<PointMass>& Mesh::getPointList() {
    return mG.getPoints();
}
