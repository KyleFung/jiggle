#include "mesh.h"

#include <GLUT/glut.h>

#include <math.h>

#include "interval.h"

Mesh::Mesh(std::string name) : mB(mG, 0), mName(name) {
    mG.setName("Geometry of " + mName);
}

Mesh::~Mesh() {
    mG.cleanUp();
}

void Mesh::setName(std::string name) {
    mName = name;
    mG.setName("Geometry of " + mName);
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

void Mesh::rotate(Eigen::Vector3f axis, float degrees) {
    axis.normalize();
    Eigen::Matrix3f rotation;
    rotation = Eigen::AngleAxisf(degToRad(degrees), axis);
    int numNodes = mG.getPoints().size();
    for (int i = 0; i < numNodes; i++) {
        PointMass& p = getPoint(i);
        p.mPos = rotation * p.mPos;
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

Collision Mesh::collide(Mesh& m, float h) {
    // Hierarchical scheme
    return mB.collide(m.getBounding(), h);
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

Geometry& Mesh::getGeometry() {
    return mG;
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
