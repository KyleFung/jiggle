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
#if OPT_BVH == 1
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
#endif
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

int Bounding::getPointBaseIndex(int i) {
    return mG.getPoints().getIndex(i);
}

int Bounding::getTriangleBaseIndex(int i) {
    return mG.getTriangles().getIndex(i);
}

int Bounding::getEdgeBaseIndex(int i) {
    return mG.getEdges().getIndex(i);
}

bool Bounding::isLeaf() {
    return !hasChildren;
}

Collision Bounding::collide(Bounding& b, float h) {
    float distance = (mCen - b.mCen).norm();
    if(distance > mRad + b.mRad) {
        return Collision();
    }

    // Base case brute force
    if(isLeaf() && b.isLeaf()) {
        Collision candidates[3];
        // Point face sweep 1
        int t0 = mG.getTriangles().size();
        int p1 = b.mG.getPoints().size();
        for(int i = 0; i < t0; i++) {
            for(int j = 0; j < p1; j++) {
                Triangle& t = getTriangle(i);
                PointMass& p = b.getPoint(j);
                int bi = getTriangleBaseIndex(i);
                int bj = b.getPointBaseIndex(j);
                Collision c = t.collide(p, h);
                float time = c.t;
                if(time != -1 && (time < candidates[0].t || candidates[0].t == -1)) {
                    candidates[0] = Collision(Collision::FACEPOINT, getTriangleBaseIndex(i), b.getPointBaseIndex(j), time, c.pos, t.getNormal());
                }
            }
        }

        // Point face sweep 2
        int t1 = b.mG.getTriangles().size();
        int p0 = mG.getPoints().size();
        for(int i = 0; i < t1; i++) {
            for(int j = 0; j < p0; j++) {
                Triangle& t = b.getTriangle(i);
                PointMass& p = getPoint(j);
                int bj = getPointBaseIndex(j);
                int bi = b.getTriangleBaseIndex(i);
                Collision c = t.collide(p, h);
                float time = c.t;
                if (time != -1 && (time < candidates[1].t || candidates[1].t == -1)) {
                    candidates[1] = Collision(Collision::POINTFACE, getPointBaseIndex(j), b.getTriangleBaseIndex(i), time, c.pos, t.getNormal());
                }
            }
        }

        // Edge edge sweep
        int e0 = mG.getEdges().size();
        int e1 = b.mG.getEdges().size();
        for(int i = 0; i < e0; i++) {
            for(int j = 0; j < e1; j++) {
                Edge& e1 = getEdge(i);
                Edge& e2 = b.getEdge(j);
                Collision c = e1.collide(e2, h);
                float time = c.t;
                if (time != -1 && (time < candidates[2].t || candidates[2].t == -1)) {
                    candidates[2] = Collision(Collision::EDGEEDGE, getEdgeBaseIndex(i), b.getEdgeBaseIndex(j), time, c.pos, e1.getNormal(e2));
                }
            }
        }

        Collision result = candidates[0];
        for(int i = 1; i < 3; i++) {
            if(candidates[i].t != -1 && (candidates[i].t < result.t || result.t == -1)) {
                result = candidates[i];
            }
        }
        return result;
    }

    // Recurse
    Collision oldest;
    for(int i = 0; i < childCount; i++) {
        Collision c = mBC[i] ? b.collide(*mBC[i], h) : Collision();
        if(c.exists() && (c.t < oldest.t || !oldest.exists())) {
            c.flip();
            oldest = c;
        }
    }

    return oldest;
}
