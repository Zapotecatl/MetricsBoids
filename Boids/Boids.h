#ifndef CBOID_H
#define CBOID_H

#include <OpenSteer/SimpleVehicle.h>
#include <OpenSteer/OpenSteerDemo.h>
#include <OpenSteer/Proximity.h>
#include <OpenSteer/UnusedParameter.h>

#include "BaseApplication.h"
#include "metrics.h"

#define GUIDES 1

using namespace OpenSteer;

typedef AbstractProximityDatabase<AbstractVehicle*> ProximityDatabase;
typedef AbstractTokenForProximityDatabase<AbstractVehicle*> ProximityToken;


class CBoid : public OpenSteer::SimpleVehicle
{
public:
    CBoid();
    CBoid (ProximityDatabase& pd);
    ~CBoid ();

    void update (const float currentTime, const float elapsedTime); // per frame simulation update
    void reset (void); // reset state
    void newPD (ProximityDatabase& pd); // switch to new proximity database

    Vec3 steerToFlock (const float currentTime, const float elapsedTime); // basic flocking
    void applySteeringForce (const Vec3& force, const float elapsedTime);
    void regenerateLocalSpaceOgre(const Vec3& newVelocity, const Vec3& oldVelocity, const float elapsedTime);

    //void ComputeRPY(void);
    Quaternion ComputeQuaternion(void);
    void BoxWrapAround (const float elapsedTime);

public:

    //OpenSteer
    typedef std::vector<CBoid*> groupType; // type for a flock: an STL vector of Boid pointers
    static ObstacleGroup obstacles; // group of all obstacles to be avoided by each Boid
    ProximityToken* proximityToken; // a pointer to this boid's interface object for the proximity database

    // allocate one and share amoung instances just to save memory usage
    // (change to per-instance allocation to be more MP-safe)
    static AVGroup neighbors;

    //Ogre 3D
    char mName_e[16];
    char mName_n[16];
    short    m_id; // member individual ID

    Entity* mEntity;
    SceneNode* mNodeBoid;

    Vector3 m_vel;
    Vector3 m_oldvel;
    Vector3 m_ang;
    Vector3 m_old_ang;
    Vector3 m_pos;

    char mName_linee[30];
    char mName_linen[30];
    char mName_linema[30];

    SceneManager *mSceneMgr;

#ifndef NO_LQ_BIN_STATS
    static size_t minNeighbors, maxNeighbors, totalNeighbors;
#endif // NO_LQ_BIN_STATS

    bool peligro;
    Vec3 mEscape;
    float mDistancia;
    Vec3 mEvade;
    FILE *m_fp_pit;

    float yaw;
    float pitch;
    float roll;

    static float worldRadius;
    float Width;
    float Heigth;
    float Length;

    float separationRadius;
    float separationAngle;
    float separationWeight;

    float alignmentRadius;
    float alignmentAngle;
    float alignmentWeight;

    float cohesionRadius;
    float cohesionAngle;
    float cohesionWeight;

};

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief The CFlock class
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


class CFlock //: public PlugIn
{

public:
    CFlock();
    ~CFlock();

    void open (SceneManager *sceneMgr, CMetrics* ptr_metrics);
    void update (const float currentTime, const float elapsedTime);
    void close (void);
    void reset (void);

    void addBoidToFlock (void);
    void proximityDatabase(void);
    void initObstacles (void);
    void updateObstacles (void); // update Boid::obstacles list when constraint changes
    void removeBoidFromFlock (void);

    RectangleObstacle bigRectangle;
    BoxObstacle outsideBigBox, insideBigBox;
    SphereObstacle insideBigSphere, outsideSphere0, outsideSphere1, outsideSphere2, outsideSphere3, outsideSphere4, outsideSphere5, outsideSphere6;

public:

    CBoid::groupType group_flock;
    typedef CBoid::groupType::const_iterator iterator;

    // enumerate demos of various constraints on the flock
    enum ConstraintType {none, insideSphere, outsideSphere, outsideSpheres, outsideSpheresNoBig, rectangle, rectangleNoBig, outsideBox, insideBox};
    ConstraintType constraint;

private:

    //OpenSteer
    ProximityDatabase* pd; // pointer to database used to accelerate proximity queries

    int population; // keep track of current flock size
    CMetrics* metrics;

};


#endif // CBOID_H
