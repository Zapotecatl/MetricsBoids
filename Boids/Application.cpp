#include "Application.h"
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */

#include <QDebug>

Application::Application(void)
{

    srand (time(NULL));

    Width_pond = 300.0;
    Heigth_pond = 150.0;
    Length_pond = 300.0;

    n = 50;

    dis_sep = 20;
    dis_ali = 30;

    d_separation = dis_sep;
    d_alignment = dis_ali;
    d_cohesion = 50;

    float ca = sqrt(Width_pond * Width_pond + Length_pond * Length_pond);
    dis_extreme_points = sqrt(ca * ca + Heigth_pond * Heigth_pond) / 2.0;

    step_sep = 10;
    step_ali = 10;
    iterations = 1;
    lim_iterations = 100000;//Lim
    experiments = 1;
    lim_experiments = 15;//Lim
    open_file = true;

    mkdir("Experiments", S_IRWXU);

}

Application::~Application(void)
{

}

bool Application::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
    bool ret = BaseApplication::frameRenderingQueued(evt);

    clock_ogre.update();


    //Metrics //////////////////////////////////////////

    Vec3 cen;
    float ext;
    Vec3 mu;
    float pol;
    int col;
    float fre_col;
    float cns_ext;
    float cns_pol;
    float quality;


    cen = metrics.calculateCenter();
    ext = metrics.calculateExtension();
    mu = metrics.calculateMu();
    pol = metrics.calculatePolarization();
    col = metrics.calculateCollisions();
    fre_col = metrics.calculateFrequencyCollision();

    metrics.setMaxExtension(dis_extreme_points);
    metrics.setPenaltyK(dis_extreme_points);
    cns_ext = metrics.calculateConsistencyExtension();

    metrics.setPenaltyRho(180.0);
    cns_pol = metrics.calculateConsistencyPolarization();

    quality = metrics.calculateQuality(0.5, 0.5);

    metrics.total_ext+=ext;
    metrics.total_pol+=pol;
    metrics.total_fre_col+=fre_col;
    metrics.total_cns_ext+=cns_ext;
    metrics.total_cns_pol+=cns_pol;
    metrics.total_quality+=quality;

    //qDebug() << "Center: " << cen.x << " " << cen.y << " " << cen.z;
    //qDebug() << "Extension: " << ext;
    //qDebug() << "Average Orientation: " << mu.x << " " << mu.y << " " << mu.z;
    //qDebug() << "Polarization: " << pol;
    //qDebug() << "Number of collision state: " << col;
    //qDebug() << "FrequencyCollision: " << fre_col;
    //qDebug() << "Consistency of extension: " << cns_ext;
    //qDebug() << "Consistency of polarizarion: " << cns_pol;

    //qDebug() << "Quality: " << quality;


#if  EXPERIMENTS
    //Configuration of distances and write in files

    if (iterations == lim_iterations) {

        metrics.total_ext/=lim_iterations;
        metrics.total_pol/=lim_iterations;
        metrics.total_fre_col/=lim_iterations;
        metrics.total_cns_ext/=lim_iterations;
        metrics.total_cns_pol/=lim_iterations;
        metrics.total_quality/=lim_iterations;

        metrics.average_ext+= metrics.total_ext;
        metrics.average_pol+= metrics.total_pol;
        metrics.average_fre_col+= metrics.total_fre_col;
        metrics.average_cns_ext+= metrics.total_cns_ext;
        metrics.average_cns_pol+= metrics.total_cns_pol;
        metrics.average_quality+= metrics.total_quality;

        metrics.total_ext = 0.0;
        metrics.total_pol = 0.0;
        metrics.total_fre_col = 0.0;
        metrics.total_cns_ext = 0.0;
        metrics.total_cns_pol = 0.0;
        metrics.total_quality = 0.0;

        if (experiments == lim_experiments) {

            if (open_file == true) {
                //printf("Abrir archivo\n");
                char str_dis[30];
                path = "Experiments/sep";
                sprintf(str_dis, "%d", dis_sep);
                path+= str_dis;
                path+= ".csv";

                file = fopen(path.data(), "w");
                fprintf(file, "Alignment, Extension, Polarization, FCollisions, CExtension, CPolarization, Quality\n", metrics.average_ext);

                open_file = false;
            }


            metrics.average_ext /= lim_experiments;
            metrics.average_pol /= lim_experiments;
            metrics.average_fre_col /= lim_experiments;
            metrics.average_cns_ext /= lim_experiments;
            metrics.average_cns_pol /= lim_experiments;
            metrics.average_quality /= lim_experiments;


            fprintf(file, "%f, ", d_alignment);
            fprintf(file, "%f, ", metrics.average_ext);
            fprintf(file, "%f, ", metrics.average_pol);
            fprintf(file, "%f, ", metrics.average_fre_col);
            fprintf(file, "%f, ", metrics.average_cns_ext);
            fprintf(file, "%f, ", metrics.average_cns_pol);
            fprintf(file, "%f\n", metrics.average_quality);

            metrics.average_ext = 0.0;
            metrics.average_pol = 0.0;
            metrics.average_fre_col = 0.0;
            metrics.average_cns_ext = 0.0;
            metrics.average_cns_pol = 0.0;
            metrics.average_quality = 0.0;

            experiments = 1;

            if (dis_sep <= 50){
                if (dis_ali < 50)
                    dis_ali+=step_ali;
                else{
                    dis_ali = 0;
                    dis_sep+=step_sep;

                    fclose(file);
                    open_file = true;
                }
            }

            d_cohesion = 50;
            d_separation = dis_sep;
            d_alignment = dis_ali;

            if (dis_sep > 50)
                return false;


        }
        else
          experiments++;

        iterations = 1;

        mFlock.reset();

    }



    iterations++;

#endif


    //Execute step of simulation //////////////////////////////////////////

    Vec3 cam = cen;
    mCamera->lookAt(Vector3(cam.x, cam.y, cam.z));

    mFlock.update(clock_ogre.getTotalSimulationTime (), clock_ogre.getElapsedSimulationTime());



    return ret;

}


void Application::createScene(void)
{

      ColourValue fadeColour(0.8, 0.8, 0.8);
      mSceneMgr->setAmbientLight(fadeColour);
      mWindow->getViewport(0)->setBackgroundColour(ColourValue(0, 0.5, 1));

      // Position it at 500 in Z direction
      mCamera->setPosition(Ogre::Vector3(0.0, Heigth_pond / 2.0, Length_pond / 4.0));

      // Look back along -Z
      mCamera->lookAt(Ogre::Vector3(0, Heigth_pond / 2.0, 0.0));

      //mSceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_ADDITIVE);
      //mSceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_TEXTURE_MODULATIVE);
      //mSceneMgr->setShadowTechnique(Ogre::SHADOWTYPE_STENCIL_MODULATIVE);

      //Ogre::ColourValue fadeColour(1.0, 1.0, 1.0);
      Ogre::ColourValue fog(0.1, 0.5, 0.8);
      //mSceneMgr->setFog(Ogre::FOG_LINEAR, fog, 0.0, 500, 2000);
      mSceneMgr->setFog(Ogre::FOG_EXP, fog, 0.002);

      //Create a light
      Ogre::Light* l = mSceneMgr->createLight("MainLight");
      l->setPosition(0,150,0);
      l->setCastShadows(true);

      Ogre::Light *light;

      light = mSceneMgr->createLight("Top");

      light->setType(Light::LT_POINT);
      light->setPosition(Vector3(0, Heigth_pond, 0));
      light->setDiffuseColour(1.0, 1.0, 1.0);
      light->setSpecularColour(1.0, 1.0, 1.0);
      light->setCastShadows(false);

      light = mSceneMgr->createLight("Bottom");

      light->setType(Light::LT_POINT);
      light->setPosition(Vector3(0, -Heigth_pond, 0));
      light->setDiffuseColour(1.0, 1.0, 1.0);
      light->setSpecularColour(1.0, 1.0, 1.0);
      light->setCastShadows(false);

      light = mSceneMgr->createLight("Front");

      light->setType(Light::LT_POINT);
      light->setPosition(Vector3(0, 0, Length_pond));
      light->setDiffuseColour(1.0, 1.0, 1.0);
      light->setSpecularColour(1.0, 1.0, 1.0);
      light->setCastShadows(false);

      light = mSceneMgr->createLight("Back");

      light->setType(Light::LT_POINT);
      light->setPosition(Vector3(0, 0, -Length_pond));
      light->setDiffuseColour(1.0, 1.0, 1.0);
      light->setSpecularColour(0.5, 0.5, 0.5);
      light->setCastShadows(false);

      light = mSceneMgr->createLight("Right");

      light->setType(Light::LT_POINT);
      light->setPosition(Vector3(Width_pond, 0, 0));
      light->setDiffuseColour(1.0, 1.0, 1.0);
      light->setSpecularColour(0.5, 0.5, 0.5);
      light->setCastShadows(false);

      light = mSceneMgr->createLight("Left");

      light->setType(Light::LT_POINT);
      light->setPosition(Vector3(-Width_pond, 0, 0));
      light->setDiffuseColour(1.0, 1.0, 1.0);
      light->setSpecularColour(0.5, 0.5, 0.5);
      light->setCastShadows(false);


      //Define a floor plane mesh
      Plane p;
      p.normal = Vector3::UNIT_Y;
      p.d = (Heigth_pond / 2) + 5;
      MeshManager::getSingleton().createPlane(
              "FloorPlane", ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
              p, 10000, 10000, 20, 20, true, 1, 50, 50, Vector3::UNIT_Z);

      // Create an entity (the floor)
      mFloor = mSceneMgr->createEntity("floor", "FloorPlane");
      mFloor->setMaterialName("suelo");
      mFloor->setCastShadows(false);
      mSceneMgr->getRootSceneNode()->createChildSceneNode()->attachObject(mFloor);

      mBox = mSceneMgr->createEntity("Box", "Box.mesh");
      mBox->setCastShadows(false);
      nodeBox = mSceneMgr->getRootSceneNode()->createChildSceneNode("NodeBox", Vector3(0, 0, 0));
      nodeBox->attachObject(mBox);
      nodeBox->scale(Width_pond, Heigth_pond, Length_pond);

      //mSceneMgr->setSkyBox(true, "Examples/SpaceSkyBox");
      mSceneMgr->setSkyDome(true, "Examples/CloudySky");

      metrics.setUpExperiments(n, lim_iterations, lim_experiments);
      mFlock.open(mSceneMgr, &metrics);

}


#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
    INT WINAPI WinMain(HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT)
#else
    int main(int argc, char *argv[])
#endif
    {
    // Create application object
    Application app;

    try {
        app.go();
    } catch(Ogre::Exception& e)  {
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
        MessageBox(NULL, e.getFullDescription().c_str(), "An exception has occurred!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else
        std::cerr << "An exception has occurred: " <<
        e.getFullDescription().c_str() << std::endl;
#endif
    }

    return 0;
    }

#ifdef __cplusplus
}
#endif

