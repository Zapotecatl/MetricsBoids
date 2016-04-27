/*
* Description: Measurements of coordinated motion in flocks are necessary to evaluate their performance. In this work, a set of
* quantitative metrics to evaluate the performance of the spatial features exhibited by flocks are introduced and
* applied to the well-known boids of Reynolds. Our metrics are based on quantitative indicators that have been used
* to evaluate fish schools. These indicators are revisited and extended as a set of three new metrics that can be
* used to evaluate and design flocks.
* *
* Author: Jorge Luis Zapotecatl Lopez
*/

#ifndef CMETRICS_H
#define CMETRICS_H

#include <OpenSteer/Vec3.h>

#include<QDebug>

#define PI 3.14159265

using namespace OpenSteer;

class CMetrics
{
public:
    CMetrics(int n = 15, int n_iter = 1000, int n_exp = 1);
    ~CMetrics();

    void setUpExperiments(int n, int n_iter, int n_exp);

    //Extension
    Vec3 calculateCenter();
    float calculateExtension();

    //Polarization
    Vec3 calculateMu();
    float calculatePolarization();

    //Frequency of collision
    float calculateCollisions();
    float calculateFrequencyCollision();

    //Proposed Metrics
    float calculateConsistencyExtension();
    float calculateConsistencyPolarization();
    float calculateQuality(float sigma, float gamma);

    void setPosistion(Vec3 &pos, int id);
    void setHeadings(Vec3 &head, int id);
    void setCollisionState(bool state, int id);
    void setPenaltyK(float k);
    void setPenaltyRho(float rho);
    void setMaxExtension(float max);

    int getNumN() const;

public:

    int num_iterations;
    int num_exp;

public:

    int num_n; //number of boids
    //int num_m; //number of boids that are not colliding
    float k_pen; //distance penalty
    float rho_pen; //distance penalty

    float max_ext;

    Vec3* positions;
    Vec3* headings;
    bool* col_state;

    //Extension
    Vec3 cen; //The center of the flock
    float ext; //The extension reflects the degree of cohesion of the flock
    float total_ext;
    float average_ext;

    //Polarization
    Vec3 mu; //The average orientation of the flock
    float pol; //Expresses the degree of alignment of the boids headings
    float total_pol;
    float average_pol;

    //Frequency of collision
    int col; //Number of boid in collision state
    float fre_col;//The frequency of collision represents the degree of conflict among boids
    float total_fre_col;
    float average_fre_col;

    //Proposed Metrics
    float cns_ext; //consistency in extension aims at balancing the distance separation in a formation
    float total_cns_ext;
    float average_cns_ext;

    float cns_pol; //consistency in polarization aims at balancing the orientation of a flock
    float total_cns_pol;
    float average_cns_pol;


    //quality aims at establishing a criterion to combine the results in the consistency in both extension and polarization,
    //in such a way that we can evaluate the global performance of a flock.
    float quality;
    float total_quality;
    float average_quality;

};

#endif // CMETRICS_H
