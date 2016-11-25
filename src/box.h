#ifndef BOX_H
#define BOX_H

#include "mesh.h"

class Box : public Mesh {
  public:
    Box();
    Box(Eigen::Vector3f pos);
    void draw();

  private:
    void init();
};

#endif
