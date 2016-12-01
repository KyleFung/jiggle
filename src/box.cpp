#include "box.h"

#include <GLUT/glut.h>

Box::Box(Eigen::Vector3f pos) {
    init();
    translate(pos);
    refreshBounding(0);
}

Box::Box() {
    init();
    refreshBounding(0);
}

void Box::draw() {
    // Draw faces
    glBegin(GL_QUADS);
    glColor4f(0.7, 0.0, 0.0, 1.0);
    glVertex3f(mP[0].mPos(0), mP[0].mPos(1), mP[0].mPos(2));
    glVertex3f(mP[1].mPos(0), mP[1].mPos(1), mP[1].mPos(2));
    glVertex3f(mP[2].mPos(0), mP[2].mPos(1), mP[2].mPos(2));
    glVertex3f(mP[3].mPos(0), mP[3].mPos(1), mP[3].mPos(2));

    glColor4f(0.0, 0.7, 0.0, 1.0);
    glVertex3f(mP[0].mPos(0), mP[0].mPos(1), mP[0].mPos(2));
    glVertex3f(mP[1].mPos(0), mP[1].mPos(1), mP[1].mPos(2));
    glVertex3f(mP[5].mPos(0), mP[5].mPos(1), mP[5].mPos(2));
    glVertex3f(mP[4].mPos(0), mP[4].mPos(1), mP[4].mPos(2));

    glColor4f(0.0, 0.0, 0.7, 1.0);
    glVertex3f(mP[4].mPos(0), mP[4].mPos(1), mP[4].mPos(2));
    glVertex3f(mP[5].mPos(0), mP[5].mPos(1), mP[5].mPos(2));
    glVertex3f(mP[6].mPos(0), mP[6].mPos(1), mP[6].mPos(2));
    glVertex3f(mP[7].mPos(0), mP[7].mPos(1), mP[7].mPos(2));

    glColor4f(0.0, 0.7, 0.7, 1.0);
    glVertex3f(mP[2].mPos(0), mP[2].mPos(1), mP[2].mPos(2));
    glVertex3f(mP[3].mPos(0), mP[3].mPos(1), mP[3].mPos(2));
    glVertex3f(mP[7].mPos(0), mP[7].mPos(1), mP[7].mPos(2));
    glVertex3f(mP[6].mPos(0), mP[6].mPos(1), mP[6].mPos(2));

    glColor4f(0.7, 0.0, 0.7, 1.0);
    glVertex3f(mP[0].mPos(0), mP[0].mPos(1), mP[0].mPos(2));
    glVertex3f(mP[3].mPos(0), mP[3].mPos(1), mP[3].mPos(2));
    glVertex3f(mP[7].mPos(0), mP[7].mPos(1), mP[7].mPos(2));
    glVertex3f(mP[4].mPos(0), mP[4].mPos(1), mP[4].mPos(2));

    glColor4f(0.7, 0.7, 0.0, 1.0);
    glVertex3f(mP[1].mPos(0), mP[1].mPos(1), mP[1].mPos(2));
    glVertex3f(mP[2].mPos(0), mP[2].mPos(1), mP[2].mPos(2));
    glVertex3f(mP[6].mPos(0), mP[6].mPos(1), mP[6].mPos(2));
    glVertex3f(mP[5].mPos(0), mP[5].mPos(1), mP[5].mPos(2));

    glColor4f(0.0, 0.0, 0.0, 1.0);
    glEnd();
}

void Box::init() {
    mK = 300;

    // Add the vertices that make up a box
    // Add corners
    addPoint(PointMass(-1, +1, +1));
    addPoint(PointMass(-1, +1, -1));
    addPoint(PointMass(+1, +1, -1));
    addPoint(PointMass(+1, +1, +1));
    addPoint(PointMass(-1, -1, +1));
    addPoint(PointMass(-1, -1, -1));
    addPoint(PointMass(+1, -1, -1));
    addPoint(PointMass(+1, -1, +1));
    // Add face centers
    addPoint(PointMass(1, 0, 0));
    addPoint(PointMass(0, 1, 0));
    addPoint(PointMass(0, 0, 1));
    addPoint(PointMass(-1, 0, 0));
    addPoint(PointMass(0, -1, 0));
    addPoint(PointMass(0, 0, -1));

    // Add the springs that make up the box
    // Outside edges
    addSpring(Spring(getPointList(), 0, 1, mK));
    addSpring(Spring(getPointList(), 1, 2, mK));
    addSpring(Spring(getPointList(), 2, 3, mK));
    addSpring(Spring(getPointList(), 3, 0, mK));
    addSpring(Spring(getPointList(), 0, 4, mK));
    addSpring(Spring(getPointList(), 1, 5, mK));
    addSpring(Spring(getPointList(), 2, 6, mK));
    addSpring(Spring(getPointList(), 3, 7, mK));
    addSpring(Spring(getPointList(), 4, 5, mK));
    addSpring(Spring(getPointList(), 5, 6, mK));
    addSpring(Spring(getPointList(), 6, 7, mK));
    addSpring(Spring(getPointList(), 7, 4, mK));
    // Face springs
    // Top
    addSpring(Spring(getPointList(), 0, 9, mK));
    addSpring(Spring(getPointList(), 1, 9, mK));
    addSpring(Spring(getPointList(), 2, 9, mK));
    addSpring(Spring(getPointList(), 3, 9, mK));
    // Right
    addSpring(Spring(getPointList(), 2, 8, mK));
    addSpring(Spring(getPointList(), 3, 8, mK));
    addSpring(Spring(getPointList(), 6, 8, mK));
    addSpring(Spring(getPointList(), 7, 8, mK));
    // Bottom
    addSpring(Spring(getPointList(), 4, 12, mK));
    addSpring(Spring(getPointList(), 5, 12, mK));
    addSpring(Spring(getPointList(), 6, 12, mK));
    addSpring(Spring(getPointList(), 7, 12, mK));
    // Left
    addSpring(Spring(getPointList(), 0, 11, mK));
    addSpring(Spring(getPointList(), 1, 11, mK));
    addSpring(Spring(getPointList(), 4, 11, mK));
    addSpring(Spring(getPointList(), 5, 11, mK));
    // Front
    addSpring(Spring(getPointList(), 0, 10, mK));
    addSpring(Spring(getPointList(), 3, 10, mK));
    addSpring(Spring(getPointList(), 4, 10, mK));
    addSpring(Spring(getPointList(), 7, 10, mK));
    // Back
    addSpring(Spring(getPointList(), 1, 13, mK));
    addSpring(Spring(getPointList(), 2, 13, mK));
    addSpring(Spring(getPointList(), 5, 13, mK));
    addSpring(Spring(getPointList(), 6, 13, mK));
    // Cross Springs
    addSpring(Spring(getPointList(), 0, 6, mK));
    addSpring(Spring(getPointList(), 1, 7, mK));
    addSpring(Spring(getPointList(), 2, 4, mK));
    addSpring(Spring(getPointList(), 3, 5, mK));
    addSpring(Spring(getPointList(), 8, 11, mK));
    addSpring(Spring(getPointList(), 9, 12, mK));
    addSpring(Spring(getPointList(), 10, 13, mK));
    addSpring(Spring(getPointList(), 8, 9, mK));
    addSpring(Spring(getPointList(), 9, 11, mK));
    addSpring(Spring(getPointList(), 11, 12, mK));
    addSpring(Spring(getPointList(), 12, 8, mK));
    addSpring(Spring(getPointList(), 9, 10, mK));
    addSpring(Spring(getPointList(), 10, 12, mK));
    addSpring(Spring(getPointList(), 12, 13, mK));
    addSpring(Spring(getPointList(), 13, 9, mK));
    addSpring(Spring(getPointList(), 8, 13, mK));
    addSpring(Spring(getPointList(), 13, 11, mK));
    addSpring(Spring(getPointList(), 11, 10, mK));
    addSpring(Spring(getPointList(), 10, 8, mK));

    // Create the faces
    // Top
    addTriangle(Triangle(getPointList(), 1, 0, 9));
    addTriangle(Triangle(getPointList(), 2, 1, 9));
    addTriangle(Triangle(getPointList(), 3, 2, 9));
    addTriangle(Triangle(getPointList(), 0, 3, 9));
    // Right
    addTriangle(Triangle(getPointList(), 2, 3, 8));
    addTriangle(Triangle(getPointList(), 6, 2, 8));
    addTriangle(Triangle(getPointList(), 7, 6, 8));
    addTriangle(Triangle(getPointList(), 3, 7, 8));
    // Bottom
    addTriangle(Triangle(getPointList(), 5, 6, 12));
    addTriangle(Triangle(getPointList(), 6, 7, 12));
    addTriangle(Triangle(getPointList(), 7, 4, 12));
    addTriangle(Triangle(getPointList(), 4, 5, 12));
    // Left
    addTriangle(Triangle(getPointList(), 0, 1, 11));
    addTriangle(Triangle(getPointList(), 4, 0, 11));
    addTriangle(Triangle(getPointList(), 5, 4, 11));
    addTriangle(Triangle(getPointList(), 1, 5, 11));
    // Front
    addTriangle(Triangle(getPointList(), 3, 0, 10));
    addTriangle(Triangle(getPointList(), 7, 3, 10));
    addTriangle(Triangle(getPointList(), 4, 7, 10));
    addTriangle(Triangle(getPointList(), 0, 4, 10));
    // Back
    addTriangle(Triangle(getPointList(), 1, 2, 13));
    addTriangle(Triangle(getPointList(), 5, 1, 13));
    addTriangle(Triangle(getPointList(), 6, 5, 13));
    addTriangle(Triangle(getPointList(), 2, 6, 13));

    // Add Edges
    addEdge(Edge(getPointList(), 0, 1));
    addEdge(Edge(getPointList(), 1, 2));
    addEdge(Edge(getPointList(), 2, 3));
    addEdge(Edge(getPointList(), 3, 0));
    addEdge(Edge(getPointList(), 0, 4));
    addEdge(Edge(getPointList(), 1, 5));
    addEdge(Edge(getPointList(), 2, 6));
    addEdge(Edge(getPointList(), 3, 7));
    addEdge(Edge(getPointList(), 4, 5));
    addEdge(Edge(getPointList(), 5, 6));
    addEdge(Edge(getPointList(), 6, 7));
    addEdge(Edge(getPointList(), 7, 4));
    // Face springs
    // Top
    addEdge(Edge(getPointList(), 0, 9));
    addEdge(Edge(getPointList(), 1, 9));
    addEdge(Edge(getPointList(), 2, 9));
    addEdge(Edge(getPointList(), 3, 9));
    // Right
    addEdge(Edge(getPointList(), 2, 8));
    addEdge(Edge(getPointList(), 3, 8));
    addEdge(Edge(getPointList(), 6, 8));
    addEdge(Edge(getPointList(), 7, 8));
    // Bottom
    addEdge(Edge(getPointList(), 4, 12));
    addEdge(Edge(getPointList(), 5, 12));
    addEdge(Edge(getPointList(), 6, 12));
    addEdge(Edge(getPointList(), 7, 12));
    // Left
    addEdge(Edge(getPointList(), 0, 11));
    addEdge(Edge(getPointList(), 1, 11));
    addEdge(Edge(getPointList(), 4, 11));
    addEdge(Edge(getPointList(), 5, 11));
    // Front
    addEdge(Edge(getPointList(), 0, 10));
    addEdge(Edge(getPointList(), 3, 10));
    addEdge(Edge(getPointList(), 4, 10));
    addEdge(Edge(getPointList(), 7, 10));
    // Back
    addEdge(Edge(getPointList(), 1, 13));
    addEdge(Edge(getPointList(), 2, 13));
    addEdge(Edge(getPointList(), 5, 13));
    addEdge(Edge(getPointList(), 6, 13));
}
