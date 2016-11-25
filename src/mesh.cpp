#include "mesh.h"

#include <GLUT/glut.h>
#include <math.h>

#include <iostream>

Mesh::Mesh() {}

Mesh::~Mesh() {}

void Mesh::draw() {
    int numTriangles = mT.size();
    for(int i = 0; i < numTriangles; i++) {
        mT[i].draw();
    }
}

void Mesh::applyGravity(float g, float h) {
    int numNodes = mP.size();
    for(int i = 0; i < numNodes; i++) {
        mP[i].mVel(1) -= (g * h) / mP[i].mMass;
    }
}

void Mesh::collideFloor(float level) {
    int numNodes = mP.size();
    for(int i = 0; i < numNodes; i++) {
        if(mP[i].mPos(1) <= level) {
            mP[i].mPos(1) = level + 0.001;
            mP[i].mVel(1) = std::abs(mP[i].mVel(1));
            mP[i].mVel *= 0.9;
        }
    }
}

void Mesh::simulate(float h) {
    int numSprings = mS.size();
    for(int i = 0; i < numSprings; i++) {
        mS[i].contributeImpulse(h);
    }
    int numNodes = mP.size();
    for(int i = 0; i < numNodes; i++) {
            mP[i].simulate(h);
    }
}

PointMass* Mesh::getClosestPoint(float minDist, Eigen::Vector3f p, Eigen::Vector3f dir) {
    PointMass* closest = NULL;
    float closestDepth = 10000;

    int numNodes = mP.size();
    for(int i = 0; i < numNodes; i++) {
        float depth = mP[i].calculateDepth(minDist, p, dir);
        // If the point is not behind the camera, and closer than the previous point
        if(depth > 0 && depth < closestDepth) {
            closest = &mP[i];
            closestDepth = depth;
        }
    }
    return closest;
}

void Mesh::addPoint(PointMass p) {
    mP.push_back(p);
}

void Mesh::addSpring(Spring s) {
    mS.push_back(s);
}

void Mesh::addTriangle(Triangle t) {
    mT.push_back(t);
}

void Mesh::addEdge(Edge e) {
    mE.push_back(e);
}

PointMass* Mesh::getPoint(int i) {
    return &mP[i];
}

Spring* Mesh::getSpring(int i) {
    return &mS[i];
}

Triangle* Mesh::getTriangle(int i) {
    return &mT[i];
}

Edge* Mesh::getEdge(int i) {
    return &mE[i];
}
