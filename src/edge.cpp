#include "edge.h"

#include <GLUT/glut.h>

#include "utils.h"

Edge::Edge(interval<PointMass>& p, int x0, int x1) : mP(p) {
    mV[0] = x0;
    mV[1] = x1;
}

void Edge::draw() {
    glBegin(GL_LINES);
    glColor4f(1.0, 1.0, 1.0, 1.0);
    glVertex3f(mP[mV[0]].mPos(0), mP[mV[0]].mPos(1), mP[mV[0]].mPos(2));
    glVertex3f(mP[mV[1]].mPos(0), mP[mV[1]].mPos(1), mP[mV[1]].mPos(2));
    glEnd();
}

void Edge::simulate(float h) {
    for(int i = 0; i < 2; i++) {
        mP[mV[i]].simulate(h);
    }
}

Eigen::Vector3f Edge::getPos(int i) {
    return mP[mV[i]].mPos;
}

Eigen::Vector3f Edge::getVel(int i) {
    return mP[mV[i]].mVel;
}

Eigen::Vector3f Edge::getFastest() {
    Eigen::Vector3f fastest = getVel(0);
    if(getVel(1).norm() > fastest.norm())
        fastest = getVel(1);
    return fastest;
}

Eigen::Vector3f Edge::getFurthest(Eigen::Vector3f center) {
    Eigen::Vector3f furthest = getPos(0);
    if((getPos(1) - center).norm() > (furthest - center).norm())
        furthest = getPos(1);
    return furthest;
}

Eigen::Vector3f Edge::getCentroid() {
    return (getPos(0) + getPos(1)) / 2.0f;
}

float Edge::collide(Edge ed, float h) {
    // Make a temporary copy of the current state
    PointMass temp[4];
    temp[0] = PointMass(mP[mV[0]]);
    temp[1] = PointMass(mP[mV[1]]);
    temp[2] = PointMass(ed.mP[ed.mV[0]]);
    temp[3] = PointMass(ed.mP[ed.mV[1]]);

    int tempIndex1[2];
    int tempIndex2[2];
    tempIndex1[0] = 0;
    tempIndex1[1] = 1;
    tempIndex2[0] = 2;
    tempIndex2[1] = 3;

    std::vector<PointMass> tempPoint(std::begin(temp), std::end(temp));
    std::vector<int> tempInt1(std::begin(tempIndex1), std::end(tempIndex1));
    std::vector<int> tempInt2(std::begin(tempIndex2), std::end(tempIndex2));
    interval<PointMass> tempInterval1(&tempPoint, &tempInt1, 0, 1);
    interval<PointMass> tempInterval2(&tempPoint, &tempInt2, 0, 1);
    Edge e1(tempInterval1, 0, 1);
    Edge e2(tempInterval2, 0, 1);

    // Step back in time using the temporary copy
    e1.simulate(-1 * h);
    e2.simulate(-1 * h);

    // Calculate the (3) times that they will be coplanar
    // Compute the 4 coefficients of the cubic equation
    Eigen::Vector3f a = e1.getPos(0) - e1.getPos(1);
    Eigen::Vector3f b = e1.getVel(0) - e1.getVel(1);
    Eigen::Vector3f c = e2.getPos(0) - e2.getPos(1);
    Eigen::Vector3f d = e2.getVel(0) - e2.getVel(1);
    Eigen::Vector3f e = e1.getPos(0) - e2.getPos(0);
    Eigen::Vector3f f = e1.getVel(0) - e2.getVel(0);

    Eigen::Vector3f bxc = b.cross(c);
    Eigen::Vector3f bxd = b.cross(d);
    Eigen::Vector3f axc = a.cross(c);
    Eigen::Vector3f axd = a.cross(d);

    float a0 = e.dot(axc);
    float a1 = f.dot(axc) + e.dot(bxc + axd);
    float a2 = e.dot(bxd) + f.dot(bxc + axd);
    float a3 = f.dot(bxd);

    float smallest = smallestPosRealRoot(a0, a1, a2, a3);

    if(smallest > h || smallest == -1) {
        return -1;
    }

    // Advance in time up until the collision and then
    // check for line intersection
    e1.simulate(smallest);
    e2.simulate(smallest);
    Eigen::Vector2f coord = intersect(e1.getPos(0), e1.getPos(1), e2.getPos(0), e2.getPos(1));

    for(int i = 0; i < 2; i++) {
         if(coord[i] > 1 || coord[i] < 0 || isnan(coord[i])) {
            return -1;
        }
    }

    return smallest;
}
