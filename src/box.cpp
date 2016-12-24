#include "box.h"

#include <GLUT/glut.h>

Box::Box(Eigen::Vector3f pos) {
    init();
    translate(pos);
    partitionBounding();
    refreshBounding(0);
}

Box::Box() {
    init();
    partitionBounding();
    refreshBounding(0);
}

void Box::draw() {
    // Draw faces
    glBegin(GL_QUADS);
    glColor4f(0.7, 0.0, 0.0, 1.0);
    std::vector<PointMass>& points = *getPointList().getBaseList();
    glVertex3f(points[0].mPos(0), points[0].mPos(1), points[0].mPos(2));
    glVertex3f(points[1].mPos(0), points[1].mPos(1), points[1].mPos(2));
    glVertex3f(points[2].mPos(0), points[2].mPos(1), points[2].mPos(2));
    glVertex3f(points[3].mPos(0), points[3].mPos(1), points[3].mPos(2));

    glColor4f(0.0, 0.7, 0.0, 1.0);
    glVertex3f(points[0].mPos(0), points[0].mPos(1), points[0].mPos(2));
    glVertex3f(points[1].mPos(0), points[1].mPos(1), points[1].mPos(2));
    glVertex3f(points[5].mPos(0), points[5].mPos(1), points[5].mPos(2));
    glVertex3f(points[4].mPos(0), points[4].mPos(1), points[4].mPos(2));

    glColor4f(0.0, 0.0, 0.7, 1.0);
    glVertex3f(points[4].mPos(0), points[4].mPos(1), points[4].mPos(2));
    glVertex3f(points[5].mPos(0), points[5].mPos(1), points[5].mPos(2));
    glVertex3f(points[6].mPos(0), points[6].mPos(1), points[6].mPos(2));
    glVertex3f(points[7].mPos(0), points[7].mPos(1), points[7].mPos(2));

    glColor4f(0.0, 0.7, 0.7, 1.0);
    glVertex3f(points[2].mPos(0), points[2].mPos(1), points[2].mPos(2));
    glVertex3f(points[3].mPos(0), points[3].mPos(1), points[3].mPos(2));
    glVertex3f(points[7].mPos(0), points[7].mPos(1), points[7].mPos(2));
    glVertex3f(points[6].mPos(0), points[6].mPos(1), points[6].mPos(2));

    glColor4f(0.7, 0.0, 0.7, 1.0);
    glVertex3f(points[0].mPos(0), points[0].mPos(1), points[0].mPos(2));
    glVertex3f(points[3].mPos(0), points[3].mPos(1), points[3].mPos(2));
    glVertex3f(points[7].mPos(0), points[7].mPos(1), points[7].mPos(2));
    glVertex3f(points[4].mPos(0), points[4].mPos(1), points[4].mPos(2));

    glColor4f(0.7, 0.7, 0.0, 1.0);
    glVertex3f(points[1].mPos(0), points[1].mPos(1), points[1].mPos(2));
    glVertex3f(points[2].mPos(0), points[2].mPos(1), points[2].mPos(2));
    glVertex3f(points[6].mPos(0), points[6].mPos(1), points[6].mPos(2));
    glVertex3f(points[5].mPos(0), points[5].mPos(1), points[5].mPos(2));

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

    std::vector<PointMass>& points = *getPointList().getBaseList();

    // Add the springs that make up the box
    // Outside edges
    addSpring(Spring(points, 0, 1, mK));
    addSpring(Spring(points, 1, 2, mK));
    addSpring(Spring(points, 2, 3, mK));
    addSpring(Spring(points, 3, 0, mK));
    addSpring(Spring(points, 0, 4, mK));
    addSpring(Spring(points, 1, 5, mK));
    addSpring(Spring(points, 2, 6, mK));
    addSpring(Spring(points, 3, 7, mK));
    addSpring(Spring(points, 4, 5, mK));
    addSpring(Spring(points, 5, 6, mK));
    addSpring(Spring(points, 6, 7, mK));
    addSpring(Spring(points, 7, 4, mK));
    // Face springs
    // Top
    addSpring(Spring(points, 0, 9, mK));
    addSpring(Spring(points, 1, 9, mK));
    addSpring(Spring(points, 2, 9, mK));
    addSpring(Spring(points, 3, 9, mK));
    // Right
    addSpring(Spring(points, 2, 8, mK));
    addSpring(Spring(points, 3, 8, mK));
    addSpring(Spring(points, 6, 8, mK));
    addSpring(Spring(points, 7, 8, mK));
    // Bottom
    addSpring(Spring(points, 4, 12, mK));
    addSpring(Spring(points, 5, 12, mK));
    addSpring(Spring(points, 6, 12, mK));
    addSpring(Spring(points, 7, 12, mK));
    // Left
    addSpring(Spring(points, 0, 11, mK));
    addSpring(Spring(points, 1, 11, mK));
    addSpring(Spring(points, 4, 11, mK));
    addSpring(Spring(points, 5, 11, mK));
    // Front
    addSpring(Spring(points, 0, 10, mK));
    addSpring(Spring(points, 3, 10, mK));
    addSpring(Spring(points, 4, 10, mK));
    addSpring(Spring(points, 7, 10, mK));
    // Back
    addSpring(Spring(points, 1, 13, mK));
    addSpring(Spring(points, 2, 13, mK));
    addSpring(Spring(points, 5, 13, mK));
    addSpring(Spring(points, 6, 13, mK));
    // Cross Springs
    addSpring(Spring(points, 0, 6, mK));
    addSpring(Spring(points, 1, 7, mK));
    addSpring(Spring(points, 2, 4, mK));
    addSpring(Spring(points, 3, 5, mK));
    addSpring(Spring(points, 8, 11, mK));
    addSpring(Spring(points, 9, 12, mK));
    addSpring(Spring(points, 10, 13, mK));
    addSpring(Spring(points, 8, 9, mK));
    addSpring(Spring(points, 9, 11, mK));
    addSpring(Spring(points, 11, 12, mK));
    addSpring(Spring(points, 12, 8, mK));
    addSpring(Spring(points, 9, 10, mK));
    addSpring(Spring(points, 10, 12, mK));
    addSpring(Spring(points, 12, 13, mK));
    addSpring(Spring(points, 13, 9, mK));
    addSpring(Spring(points, 8, 13, mK));
    addSpring(Spring(points, 13, 11, mK));
    addSpring(Spring(points, 11, 10, mK));
    addSpring(Spring(points, 10, 8, mK));

    // Create the faces
    // Top
    addTriangle(Triangle(points, 1, 0, 9));
    addTriangle(Triangle(points, 2, 1, 9));
    addTriangle(Triangle(points, 3, 2, 9));
    addTriangle(Triangle(points, 0, 3, 9));
    // Right
    addTriangle(Triangle(points, 2, 3, 8));
    addTriangle(Triangle(points, 6, 2, 8));
    addTriangle(Triangle(points, 7, 6, 8));
    addTriangle(Triangle(points, 3, 7, 8));
    // Bottom
    addTriangle(Triangle(points, 5, 6, 12));
    addTriangle(Triangle(points, 6, 7, 12));
    addTriangle(Triangle(points, 7, 4, 12));
    addTriangle(Triangle(points, 4, 5, 12));
    // Left
    addTriangle(Triangle(points, 0, 1, 11));
    addTriangle(Triangle(points, 4, 0, 11));
    addTriangle(Triangle(points, 5, 4, 11));
    addTriangle(Triangle(points, 1, 5, 11));
    // Front
    addTriangle(Triangle(points, 3, 0, 10));
    addTriangle(Triangle(points, 7, 3, 10));
    addTriangle(Triangle(points, 4, 7, 10));
    addTriangle(Triangle(points, 0, 4, 10));
    // Back
    addTriangle(Triangle(points, 1, 2, 13));
    addTriangle(Triangle(points, 5, 1, 13));
    addTriangle(Triangle(points, 6, 5, 13));
    addTriangle(Triangle(points, 2, 6, 13));

    // Add Edges
    addEdge(Edge(points, 0, 1));
    addEdge(Edge(points, 1, 2));
    addEdge(Edge(points, 2, 3));
    addEdge(Edge(points, 3, 0));
    addEdge(Edge(points, 0, 4));
    addEdge(Edge(points, 1, 5));
    addEdge(Edge(points, 2, 6));
    addEdge(Edge(points, 3, 7));
    addEdge(Edge(points, 4, 5));
    addEdge(Edge(points, 5, 6));
    addEdge(Edge(points, 6, 7));
    addEdge(Edge(points, 7, 4));
    // Face springs
    // Top
    addEdge(Edge(points, 0, 9));
    addEdge(Edge(points, 1, 9));
    addEdge(Edge(points, 2, 9));
    addEdge(Edge(points, 3, 9));
    // Right
    addEdge(Edge(points, 2, 8));
    addEdge(Edge(points, 3, 8));
    addEdge(Edge(points, 6, 8));
    addEdge(Edge(points, 7, 8));
    // Bottom
    addEdge(Edge(points, 4, 12));
    addEdge(Edge(points, 5, 12));
    addEdge(Edge(points, 6, 12));
    addEdge(Edge(points, 7, 12));
    // Left
    addEdge(Edge(points, 0, 11));
    addEdge(Edge(points, 1, 11));
    addEdge(Edge(points, 4, 11));
    addEdge(Edge(points, 5, 11));
    // Front
    addEdge(Edge(points, 0, 10));
    addEdge(Edge(points, 3, 10));
    addEdge(Edge(points, 4, 10));
    addEdge(Edge(points, 7, 10));
    // Back
    addEdge(Edge(points, 1, 13));
    addEdge(Edge(points, 2, 13));
    addEdge(Edge(points, 5, 13));
    addEdge(Edge(points, 6, 13));
}
