#include "Boids.h"

#include <QDebug>

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief CBoid::CBoid
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


AVGroup CBoid::neighbors;
float CBoid::worldRadius = 50.0f;
ObstacleGroup CBoid::obstacles;
#ifndef NO_LQ_BIN_STATS
size_t CBoid::minNeighbors, CBoid::maxNeighbors, CBoid::totalNeighbors;
#endif // NO_LQ_BIN_STATS

CBoid::CBoid()
{
    proximityToken = NULL;
    reset ();
}

CBoid::CBoid(ProximityDatabase& pd)
{
    // allocate a token for this boid in the proximity database
    proximityToken = NULL;
    newPD (pd);

    reset();

}

CBoid::~CBoid ()
{
    // delete this boid's token in the proximity database
    delete proximityToken;
}

// reset state
void CBoid::reset (void)
{
    // reset the vehicle
    SimpleVehicle::reset ();

    setMass(1);
    setMaxForce(50.0); // steering force is clipped to this magnitude
    setMaxSpeed(25.0); // velocity is clipped to this magnitude
    //setSpeed (maxSpeed()); // initial slow speed
    setSpeed(10); // initial slow speed

    // randomize initial orientation
    regenerateOrthonormalBasisUF (RandomUnitVector ());

    Vec3 inipos = RandomVectorInUnitRadiusSphere () * 50; // randomize initial position

    setPosition (inipos);

    proximityToken->updateForNewPosition (position()); // notify proximity database that our position has changed

    m_vel = Vector3(0, 0, 0);
    m_oldvel = Vector3(0, 0, 0);

    m_ang = Vector3(0, 0, 0);
    m_old_ang = Vector3(0, 0, 0);


    Width = Length_pond;
    Heigth = Heigth_pond;
    Length = Width_pond;

    separationRadius =  d_separation;
    separationAngle  = -1.0f;
    separationWeight =  1.0 * 25.0f;

    alignmentRadius = d_alignment;
    alignmentAngle  = -1.0f;
    alignmentWeight = 1 * 25.0f;

    cohesionRadius = d_cohesion;
    cohesionAngle  = -1.0f;
    cohesionWeight = 1.0 * 25.0f;

}

// switch to new proximity database -- just for demo purposes
void CBoid::newPD (ProximityDatabase& pd)
{
    // delete this boid's token in the old proximity database
    delete proximityToken;

    // allocate a token for this boid in the proximity database
    proximityToken = pd.allocateToken (this);


}


// per frame simulation update
void CBoid::update (const float currentTime, const float elapsedTime)
{

    OPENSTEER_UNUSED_PARAMETER(currentTime);

    Vec3 s;

    s = steerToFlock(currentTime, elapsedTime);

    applySteeringForce (s, elapsedTime); // steer to flock and avoid obstacles if any

    // wrap around to contrain boid within the spherical boundary
    //sphericalWrapAround ();
    BoxWrapAround (elapsedTime);

    // notify proximity database that our position has changed
    proximityToken->updateForNewPosition (position());


}

Vec3 CBoid::steerToFlock (const float currentTime, const float elapsedTime)
{

    Vec3 steer(0.0, 0.0, 0.0);

    steer = steerToAvoidObstacles (1.0f, obstacles);// avoid obstacles if needed

    if (steer != Vec3::zero)
        return steer;



    const float maxRadius = maxXXX (separationRadius, maxXXX (alignmentRadius,  cohesionRadius));
    // find all flockmates within maxRadius using proximity database
    neighbors.clear();
    proximityToken->findNeighbors (position(), maxRadius, neighbors);

//#ifndef NO_LQ_BIN_STATS

    // maintain stats on max/min/ave neighbors per boids
    size_t count = neighbors.size();
    if (maxNeighbors < count)
        maxNeighbors = count;

    if (minNeighbors > count)
        minNeighbors = count;

    totalNeighbors += count;
//#endif // NO_LQ_BIN_STATS

    //qDebug() << count;
    //if (count == 1) {
    //    steer = velocity();
    //    return steer;
    //}

    const Vec3 cohesion = steerForCohesion(cohesionRadius, cohesionAngle, neighbors);
    const Vec3 separation = steerForSeparation(separationRadius, separationAngle, neighbors);
    const Vec3 alignment = steerForAlignment(alignmentRadius, alignmentAngle, neighbors);

    // apply weights to components (save in variables for annotation)
    Vec3 cohesionW = cohesionWeight * cohesion;
    Vec3 separationW = separationWeight * separation;
    Vec3 alignmentW = alignmentWeight * alignment;

    steer = cohesionW + separationW + alignmentW;

    return steer;

}

void CBoid::BoxWrapAround (const float elapsedTime)
{

    float maxX = Length / 2.0;
    float maxY = Heigth / 2.0;
    float maxZ = Width / 2.0;

    float minX = -maxX;
    float minY = -maxY;
    float minZ = -maxZ;

    bool wall = false;

    Vec3 d = velocity();
    Vec3 p = position();

    if (p.x > maxX) {

        d.x = -d.x;
        p.x = maxX;
        wall = true;
    }

    if (p.x < minX) {

        d.x = -d.x;
        p.x = minX;
        wall = true;
    }

    if (p.y > maxY ) {

        d.y = -d.y;
        p.y = maxY;
        wall = true;
    }

    if (p.y < minY) {

        d.y = -d.y;
        p.y = minY;
        wall = true;
    }

    if (p.z > maxZ) {

        d.z = -d.z;
        p.z = maxZ;
        wall = true;
    }

    if (p.z < minZ) {

        d.z = -d.z;
        p.z = minZ;
        wall = true;
    }


    if (wall == true) {
        //setSpeed(d.length());
        setPosition(p);
        //applySteeringForce (50*d, elapsedTime, peligro);
        regenerateLocalSpace (d, elapsedTime);
    }

}

Quaternion CBoid::ComputeQuaternion(void)
{
      // Input matrix:
      float m11, m12, m13;
      float m21, m22, m23;
      float m31, m32, m33;

      m11 = side().x;
      m12 = side().y;
      m13 = side().z;

      m21 = up().x;
      m22 = up().y;
      m23 = up().z;

      m31 = forward().x;
      m32 = forward().y;
      m33 = forward().z;

      Matrix3 matriz(m11, m21, m31, m12, m22, m32, m13, m23, m33);
      Quaternion quat(matriz);

  return quat;

}


void CBoid::applySteeringForce (const Vec3& force, const float elapsedTime)
{

    //const Vec3 adjustedForce = adjustRawSteeringForce (force, elapsedTime);

    // enforce limit on magnitude of steering force
    //const Vec3 clippedForce = adjustedForce.truncateLength (maxForce ());

    const Vec3 clippedForce = force.truncateLength (maxForce ());


    // compute acceleration and velocity
    Vec3 newAcceleration = (clippedForce / mass());

    Vec3 oldVelocity = velocity();
    Vec3 newVelocity = velocity();

    //fprintf(m_fp_pit, "(%f, %f, %f)-", newVelocity.x, newVelocity.y, newVelocity.z);

    // damp out abrupt changes and oscillations in steering acceleration
    // (rate is proportional to time step, then clipped into useful range)
    if (elapsedTime > 0) {
        const float smoothRate = clip (9 * elapsedTime, 0.15f, 0.4f);
        blendIntoAccumulator (smoothRate, newAcceleration, _smoothedAcceleration);
    }

    // Euler integrate (per frame) acceleration into velocity
    newVelocity += _smoothedAcceleration * elapsedTime;
    //newVelocity.y = 0.0;

    //float yaw = (float) atan2((float)newVelocity.x, (float)newVelocity.y);
    //fprintf(m_fp_pit, "(%f, %f, %f) - %f\n", newVelocity.x, newVelocity.y, newVelocity.z, yaw * (180/3.1416));
    //fprintf(m_fp_pit, "(%f, %f, %f)\n", newVelocity.x, newVelocity.y, newVelocity.z);

    // enforce speed limit
    newVelocity = newVelocity.truncateLength (maxSpeed ());

    // update Speed
    setSpeed (newVelocity.length());

    // Euler integrate (per frame) velocity into position
    setPosition (position() + (newVelocity * elapsedTime));

    // regenerate local space (by default: align vehicle's forward axis with
    // new velocity, but this behavior may be overridden by derived classes.)
    //regenerateLocalSpace (newVelocity, elapsedTime);

    regenerateLocalSpaceOgre(newVelocity, oldVelocity, elapsedTime);

    // maintain path curvature information
    // measurePathCurvature (elapsedTime);

    // running average of recent positions
    // blendIntoAccumulator (elapsedTime * 0.06f, position(), _smoothedPosition);

}


void CBoid::regenerateLocalSpaceOgre(const Vec3& newVelocity, const Vec3& oldVelocity, const float elapsedTime)
{

    float sy,cy, sp,cp, sr,cr;

    roll = pitch = yaw = 0;

    // Determine the direction of the lateral acceleration.
    Vec3 lateralDir;
    Vec3 aux, aux2;

    aux.cross(newVelocity, newVelocity - oldVelocity);
    aux2.cross(aux, newVelocity);
    lateralDir = aux2.normalize();

    // Set the lateral acceleration's magnitude. The magnitude is the vector
    // projection of the appliedAcceleration vector onto the direction of the
    // lateral acceleration).

    float lateralMag;

    aux = newVelocity - oldVelocity;
    lateralMag = aux.dot(lateralDir);

    //fprintf(m_fp_pit, "(%f, %f, %f)-", newVelocity.x, newVelocity.y, newVelocity.z);

    // compute yaw
    yaw = (float) atan2((float)newVelocity.x, (float)newVelocity.z);

    //compute pitch
    if (newVelocity.z != 0.0 && newVelocity.x != 0.0)
        pitch = (float) -atan((float) (newVelocity.y / sqrt((float)(newVelocity.z * newVelocity.z + newVelocity.x * newVelocity.x))));
    else
        pitch = 0.0;

    // compute roll
    if (lateralMag == 0.0)
        roll = 0.0f;
    else
        roll = (float) -atan2(9.806650f, lateralMag) + 1.57079f;


    //// Rotation matrix /////////////////////////////////////////

    sy = sin(yaw);
    cy = cos(yaw);

    sp = sin(pitch);
    cp = cos(pitch);

    sr = sin(roll);
    cr = cos(roll);

    // Input matrix:
    float m11, m12, m13;
    float m21, m22, m23;
    float m31, m32, m33;

    // ////YPR////////////////////

    m11 = cy * cr + sy * sp * sr;
    m12 = -cy * sr + sy * sp * cr;
    m13 = sy * cp;

    m21 = sr * cp;
    m22 = cr * cp;
    m23 = -sp;

    m31 = -sy * cr + cy * sp * sr;
    m32 = sr * sy + cy * sp * cr;
    m33 = cy * cp;


    setSide(m11, m21, m31);
    setUp(m12, m22, m32);
    setForward(m13, m23, m33);


}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/// \brief CFlock::CFlock
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CFlock::CFlock()
{

    population = 0;
    pd = NULL;


}

CFlock::~CFlock()
{

}

void CFlock::addBoidToFlock (void)
{
    population++;

    CBoid* ptrBoid = new CBoid(*pd);

    group_flock.push_back (ptrBoid);

    if (population == 1)
        OpenSteerDemo::selectedVehicle = ptrBoid;

}


void CFlock::open(SceneManager *sceneMgr, CMetrics *ptr_metrics)
{

    metrics = ptr_metrics;

    proximityDatabase();

    population = 0; // make default-sized flock

    for (int i = 0; i < metrics->getNumN(); i++)
        addBoidToFlock ();

    int id = 0;
    for (iterator i = group_flock.begin(); i != group_flock.end(); i++) {


        sprintf((**i).mName_linee, "Line%d%d", id, id);
        sprintf((**i).mName_linen, "Line%dNode%d", id, id);
        sprintf((**i).mName_linema, "Material%d%d", id, id);

        (**i).mSceneMgr = sceneMgr;

        sprintf((**i).mName_e, "Pez%d", id);
        sprintf((**i).mName_n, "Pez%dNode", id);

        (**i).mEntity = sceneMgr->createEntity((**i).mName_e, "arenque.mesh");
        (**i).mEntity->setCastShadows(true);
        (**i).m_id = id;

        Vec3 p = (**i).position();
        Vec3 h = (**i).forward();

        Vector3 pos;

        pos.x = p.x;
        pos.y = p.y;
        pos.z = p.z;

        (**i).mNodeBoid = sceneMgr->getRootSceneNode()->createChildSceneNode((**i).mName_n, pos);
        (**i).mNodeBoid->attachObject((**i).mEntity);

        //(**i).mNodeBoid->scale(0.3, 0.3, 0.3);
        //(**i).mNodeBoid->setVisible(false);


#if GUIDES
        char nombreeje[100];

        sprintf(nombreeje, "MyAxis%d", id);

        ManualObject *mAxis = sceneMgr->createManualObject(nombreeje);
        mAxis->setCastShadows(false);
        mAxis->begin( "BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST );
        /*mAxis->position(0,0,0);
        mAxis->colour(1,0,0);
        mAxis->position(25,0,0);
        mAxis->position(0,0,0);
        mAxis->colour(0,0,1);
        mAxis->position(0,25,0);*/
        mAxis->position(0,0,0);
        mAxis->colour(0,1,0);
        mAxis->position(0,0,10);
        mAxis->end();

        (**i).mNodeBoid->attachObject(mAxis);
#endif

        metrics->setPosistion(p, id);
        metrics->setHeadings(h, id);

        id++;

    }

    initObstacles (); // set up obstacles


}


void CFlock::update (const float currentTime, const float elapsedTime)
{

    int id = 0;
    for (iterator i = group_flock.begin(); i != group_flock.end(); i++)  {

        Vec3 vold = (**i).velocity();
        (**i).m_oldvel = (**i).m_vel;

        (**i).update(currentTime, elapsedTime);

        Vec3 v = (**i).velocity();

        (**i).m_vel.x = v.x;
        (**i).m_vel.y = v.y;
        (**i).m_vel.z = v.z;

        Vec3 p = (**i).position();
        Vec3 h = (**i).forward();

        Vector3 pos;

        pos.x = p.x;
        pos.y = p.y;
        pos.z = p.z;

        (**i).m_pos.x = p.x;
        (**i).m_pos.y = p.y;
        (**i).m_pos.z = p.z;

        (**i).mNodeBoid->setPosition(pos);

        Quaternion quatboid = (**i).ComputeQuaternion();
        (**i).mNodeBoid->setOrientation(quatboid);

        /*
        ang_yaw+= fabs((**i).yaw * (180 / PI));
        ang_pitch+= fabs((**i).pitch * (180 / PI));
        ang_roll+= fabs((**i).roll * (180 / PI));*/

        metrics->setPosistion(p, id);
        metrics->setHeadings(h, id);

        id++;

    }

}

void CFlock::close (void)
{

    // delete each member of the flock
    while (population > 0)
        removeBoidFromFlock ();

    // delete the proximity database
    delete pd;

    pd = NULL;

}

void CFlock::removeBoidFromFlock (void)
{
    if (population > 0) {

        // save a pointer to the last boid, then remove it from the flock
        const CBoid* boid = group_flock.back();

        group_flock.pop_back();
        population--;

        // if it is OpenSteerDemo's selected vehicle, unselect it
        if (boid == OpenSteerDemo::selectedVehicle)
            OpenSteerDemo::selectedVehicle = NULL;

        // delete the Boid
        delete boid;
    }
}

void CFlock::reset (void)
{

    for (iterator i = group_flock.begin(); i != group_flock.end(); i++)
        (**i).reset();
}


// for purposes of demonstration, allow cycling through various types of proximity databases.
void CFlock::proximityDatabase (void)
{
    // save pointer to old PD
    ProximityDatabase* oldPD = pd;

    const Vec3 center;
    const float div = 10.0f;
    const Vec3 divisions (div, div, div);
    const float diameter = CBoid::worldRadius * 1.1f * 2;
    const Vec3 dimensions (diameter, diameter, diameter);
    typedef LQProximityDatabase<AbstractVehicle*> LQPDAV;
    pd = new LQPDAV (center, dimensions, divisions);

    /*
    // allocate new PD
    const int totalPD = 2;
    switch (cyclePD = (cyclePD + 1) % totalPD)
    {
    case 0:
        {
            const Vec3 center;
            const float div = 10.0f;
            const Vec3 divisions (div, div, div);
            const float diameter = Boid::worldRadius * 1.1f * 2;
            const Vec3 dimensions (diameter, diameter, diameter);
            typedef LQProximityDatabase<AbstractVehicle*> LQPDAV;
            pd = new LQPDAV (center, dimensions, divisions);

            break;
        }
    case 1:
        {
            pd = new BruteForceProximityDatabase<AbstractVehicle*> ();

            break;
        }
    }
    */

    for (iterator i = group_flock.begin(); i != group_flock.end(); i++) // switch each boid to new PD
        (**i).newPD(*pd);

    delete oldPD; // delete old PD (if any)

}

void CFlock::initObstacles (void)
{
    //constraint = none;


    constraint = insideBox;

    insideBigSphere.radius = CBoid::worldRadius;
    insideBigSphere.setSeenFrom (Obstacle::inside);

    outsideSphere0.radius = CBoid::worldRadius * 0.5f;

    const float r = CBoid::worldRadius * 0.33f;
    outsideSphere1.radius = r;
    outsideSphere2.radius = r;
    outsideSphere3.radius = r;
    outsideSphere4.radius = r;
    outsideSphere5.radius = r;
    outsideSphere6.radius = r;

    const float p = CBoid::worldRadius * 0.5f;
    const float m = -p;
    const float z = 0.0f;
    outsideSphere1.center.set (p, z, z);
    outsideSphere2.center.set (m, z, z);
    outsideSphere3.center.set (z, p, z);
    outsideSphere4.center.set (z, m, z);
    outsideSphere5.center.set (z, z, p);
    outsideSphere6.center.set (z, z, m);

    /*const Vec3 tiltF = Vec3 (1.0f, 1.0f, 0.0f).normalize ();
    const Vec3 tiltS (0.0f, 0.0f, 1.0f);
    const Vec3 tiltU = Vec3 (-1.0f, 1.0f, 0.0f).normalize ();*/


    const Vec3 tiltF = Vec3(1.0f, 0.0f, 0.0f).normalize ();
    const Vec3 tiltS = Vec3(0.0f, 0.0f, 1.0f).normalize ();
    const Vec3 tiltU = Vec3(0.0f, 1.0f, 0.0f).normalize ();

    bigRectangle.width = 50.0f;
    bigRectangle.height = 80.0f;
    bigRectangle.setSeenFrom (Obstacle::both);
    bigRectangle.setForward (tiltF);
    bigRectangle.setSide (tiltS);
    bigRectangle.setUp (tiltU);

    outsideBigBox.width = 50.0f;
    outsideBigBox.height = 80.0f;
    outsideBigBox.depth = 20.0f;
    outsideBigBox.setForward (tiltF);
    outsideBigBox.setSide (tiltS);
    outsideBigBox.setUp (tiltU);

    outsideBigBox.width = 50.0f;
    outsideBigBox.height = 80.0f;
    outsideBigBox.depth = 20.0f;
    outsideBigBox.setForward (tiltF);
    outsideBigBox.setSide (tiltS);
    outsideBigBox.setUp (tiltU);

    insideBigBox.width = Length_pond;
    insideBigBox.height = Heigth_pond;
    insideBigBox.depth = Width_pond;

    insideBigBox.setForward (tiltF);
    insideBigBox.setSide (tiltS);
    insideBigBox.setUp (tiltU);

    insideBigBox.setSeenFrom (Obstacle::inside);

    updateObstacles ();
 }


    // update Boid::obstacles list when constraint changes
 void CFlock::updateObstacles (void)
 {
    // first clear out obstacle list
    CBoid::obstacles.clear ();

    // add back obstacles based on mode
    switch (constraint)
    {
    default:
        // reset for wrap-around, fall through to first case:
        constraint = none;
    case none:
        break;
    case insideSphere:
        CBoid::obstacles.push_back (&insideBigSphere);
        break;
    case outsideSphere:
        CBoid::obstacles.push_back (&insideBigSphere);
        CBoid::obstacles.push_back (&outsideSphere0);
        break;
    case outsideSpheres:
        CBoid::obstacles.push_back (&insideBigSphere);
    case outsideSpheresNoBig:
        CBoid::obstacles.push_back (&outsideSphere1);
        CBoid::obstacles.push_back (&outsideSphere2);
        CBoid::obstacles.push_back (&outsideSphere3);
        CBoid::obstacles.push_back (&outsideSphere4);
        CBoid::obstacles.push_back (&outsideSphere5);
        CBoid::obstacles.push_back (&outsideSphere6);
        break;
    case rectangle:
        CBoid::obstacles.push_back (&insideBigSphere);
        CBoid::obstacles.push_back (&bigRectangle);
    case rectangleNoBig:
        CBoid::obstacles.push_back (&bigRectangle);
        break;
    case outsideBox:
        CBoid::obstacles.push_back (&insideBigSphere);
        CBoid::obstacles.push_back (&outsideBigBox);
        break;
    case insideBox:
        CBoid::obstacles.push_back (&insideBigBox);

        break;
    }
}

