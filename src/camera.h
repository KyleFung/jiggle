#ifndef CAMERA_H
#define CAMERA_H

#include <Eigen/Dense>

class Camera
{
  public:
    Camera(Eigen::Vector3f pos, float pitch, float yaw,
               int windowWidth, int windowHeight, float fovy);
    bool onKey(int key);
    bool onMouse(int x, int y);
    bool updateMouse(int x, int y);
    Eigen::Vector3f getDir();
    Eigen::Vector3f getUp();
    Eigen::Vector3f unproject(int x, int y);

    Eigen::Vector3f mPos;
    float mPitch;
    float mYaw;

    int mWinWidth;
    int mWinHeight;
    float mFovy;
    bool mStarted;

  private:
    float mStepSize;
    int mMouseX;
    int mMouseY;
};

#endif
