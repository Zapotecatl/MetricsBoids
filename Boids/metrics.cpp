#include "metrics.h"

CMetrics::CMetrics(int n, int n_iter, int n_exp)
{
    num_n = 0;
    //num_m = 0; //number of boids that are not colliding
    k_pen = 0.0; //distance penalty
    rho_pen = 0.0; //angle penalty
    cen = Vec3(0.0, 0.0, 0.0); //The center of the flock
    ext = 0.0; //The extension reflects the degree of cohesion of the flock
    total_ext = 0.0;
    average_ext = 0.0;
    mu = Vec3(0.0, 0.0, 0.0); //The average orientation of the flock
    pol = 0.0; //Expresses the degree of alignment of the boids headings
    total_pol = 0.0;
    average_pol = 0.0;
    col = 0; //Number of boid in collision state
    fre_col = 0.0;//The frequency of collision represents the degree of conflict among boids
    total_fre_col = 0.0;
    average_fre_col = 0.0;
    cns_ext = 0.0; //consistency in extension aims at balancing the distance separation in a formation
    total_cns_ext = 0.0;
    average_cns_ext = 0.0;
    cns_pol = 0.0; //consistency in polarization aims at balancing the orientation of a flock
    total_cns_pol = 0.0;
    average_cns_pol = 0.0;
    quality = 0.0;
    total_quality = 0.0;
    average_quality = 0.0;

    positions = NULL;
    headings = NULL;
    col_state = NULL;

    setUpExperiments(n, n_iter, n_exp);

}

CMetrics::~CMetrics()
{
    delete [] positions;
    delete [] headings;
    delete [] col_state;
}

void CMetrics::setUpExperiments(int n, int n_iter, int n_exp)
{

    if (n <= 0)
        n = 15;

    if (num_n != n) {
        num_n = n;


        if (positions != NULL)
            delete [] positions;

        if (headings != NULL)
            delete [] headings;

        if (col_state != NULL)
            delete [] col_state;

        positions = new Vec3[num_n];
        headings = new Vec3[num_n];
        col_state = new bool[num_n];
    }

    if (n_iter <= 0)
        num_iterations = 1000;

    if (n_exp <= 0)
        num_exp = 1;

}

Vec3 CMetrics::calculateCenter()
{

    cen = Vec3(0.0, 0.0, 0.0);
    for (int i = 0; i < num_n; i++)
        cen+= positions[i];

    //qDebug() << cen.x << " " << cen.y << " " << cen.z;

    cen.x = cen.x / (float) num_n;
    cen.y = cen.y / (float) num_n;
    cen.z = cen.z / (float) num_n;

    return cen;

}

float CMetrics::calculateExtension()
{

    ext = 0.0;
    for (int i = 0; i < num_n; i++)
        ext+= (cen - positions[i]).length();

    ext = ext / (float) num_n;

    return ext;

}

Vec3 CMetrics::calculateMu()
{

    mu = Vec3(0.0, 0.0, 0.0);
    for (int i = 0; i < num_n; i++)
        mu+= headings[i];

   // qDebug() << mu.x << " " << mu.y << " " << mu.z;

    mu.x = mu.x / (float) num_n;
    mu.y = mu.y / (float) num_n;
    mu.z = mu.z / (float) num_n;

    return mu;

}

float CMetrics::calculatePolarization()
{

    float angle;
    Vec3 norm;

    pol = 0.0;
    for (int i = 0; i < num_n; i++){
       norm = mu.normalize();
       angle = headings[i].dot(norm);

       angle = angle > 1.0 ? 1.0 : angle;
       angle = angle < -1.0 ? -1.0 : angle;

       angle = acos(angle) * 180.0 / PI;

       pol+= angle;
    }

    // qDebug() << "pol: " << pol;
    pol = pol / (float) num_n;

    return pol;

}

float CMetrics::calculateCollisions()
{

    int i, j;


    float l_body = 5.0;
    float l_body_squared = l_body * l_body;

    for (i = 0; i < num_n; i++)
        col_state[i] = false;


    for (int i = 0; i < num_n - 1; i++)
        if (col_state[i] == false) {
            for (j = i + 1; j < num_n; j++)
                if ((positions[j] - positions[i]).lengthSquared() < l_body_squared)
                    col_state[i] = col_state[j] = true;
        }

    col = 0;
    for (i = 0; i < num_n; i++)
        if (col_state[i] == true)
            col++;

    return col;

}

float CMetrics::calculateFrequencyCollision()
{

    fre_col = (float) col / (float) num_n;

    return fre_col;

}

float CMetrics::calculateConsistencyExtension()
{

    float sum_m;
    float penalty;

    sum_m = 0.0;
    for (int i = 0; i < num_n; i++)
        if (col_state[i] == false)
            sum_m+= (cen - positions[i]).length();

    penalty = k_pen * col;

    cns_ext = 1.0 - ((sum_m + penalty) / (max_ext * num_n));

    return cns_ext;

}

float CMetrics::calculateConsistencyPolarization()
{

    float sum_m;
    float angle;
    float penalty;
    Vec3 norm;

    sum_m = 0.0;
    for (int i = 0; i < num_n; i++)
        if (col_state[i] == false) {
            norm = mu.normalize();
            angle = headings[i].dot(norm);

            angle = angle > 1.0 ? 1.0 : angle;
            angle = angle < -1.0 ? -1.0 : angle;

            angle = acos(angle) * 180.0 / PI;

            sum_m+= angle;
        }

    penalty = rho_pen * col;

    cns_pol = 1.0 - ((sum_m + penalty) / (180.0 * num_n));

    return cns_pol;


}

float CMetrics::calculateQuality(float sigma, float gamma)
{

    if ((sigma + gamma) != 1.0)
       sigma = gamma = 0.5;

    quality = sigma * cns_ext + gamma * cns_pol;

    return quality;

}

void CMetrics::setPosistion(Vec3 &pos, int id)
{
   id = (id < num_n && id >= 0) ? id : 0;
   positions[id] = pos;
}

void CMetrics::setHeadings(Vec3 &head, int id)
{
    id = (id < num_n && id >= 0) ? id : 0;
    headings[id] = head;
}

void CMetrics::setCollisionState(bool state, int id)
{
    id = (id < num_n && id >= 0) ? id : 0;
    col_state[id] = state;
}

void CMetrics::setMaxExtension(float max)
{

    if (max <= 0)
       max = 1;

    max_ext = max;

}

void CMetrics::setPenaltyK(float k)
{
    if (k < 0)
        k = 0;
    else if (k > max_ext)
        k =  max_ext;

    k_pen = k;
}

void CMetrics::setPenaltyRho(float rho)
{

    if (rho < 0)
        rho = 0;
    else if (rho > 180)
        rho =  180;

    rho_pen = rho;
}

int CMetrics::getNumN() const
{
    return num_n;
}


