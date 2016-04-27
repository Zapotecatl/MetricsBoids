#ifndef __Application_h_
#define __Application_h_

#include <string>

#include "BaseApplication.h"
#include "Boids.h"

#define EXPERIMENTS 0

OpenSteer::Clock clock_ogre;

class Application : public BaseApplication
{
public:
  Application();
  virtual ~Application();


  virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);

public:

  CFlock mFlock;

  Entity *mFloor, *mBox;
  SceneNode* nodeFloor, *nodeBox;

  float dis_extreme_points;

  int n;
  int experiments;
  int lim_experiments;
  int iterations;
  int lim_iterations;

  int dis_sep;
  int step_sep;
  int dis_ali;
  int step_ali;

  FILE *file;
  bool open_file;
  std::string path;

  bool finished_exp;

protected:
  virtual void createScene();
  //virtual void createCamera();
  //virtual void createViewports();


};

#endif // #ifndef __Application_h_
