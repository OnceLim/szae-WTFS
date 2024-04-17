//
// Created by Chin Zong Han on 4/15/24.
//

#ifndef CLOTHSIM_NAVIERSTOKES_H
#define CLOTHSIM_NAVIERSTOKES_H

#include <unordered_set>
#include <unordered_map>
#include <vector>
#include "pointMass.h"

#include "CGL/CGL.h"
#include "CGL/misc.h"
using namespace std;
using namespace CGL;

class navierStokes {

private:
    vector<std::vector<Vector3D>> velocityField;
    vector<std::vector<double>> pressureField;
    double viscosity; //might not be required
    double globalDensity;

    void CheckParticleDistance(PointMass &pm);
    void CalculateForces(PointMass &pm);

public:
    void computePressure(PointMass &pm);
    void computeViscosity(PointMass &pm);
    void applyForces(PointMass &pm);
    double smoothingKernel(double distance);


};



#endif //CLOTHSIM_NAVIERSTOKES_H
