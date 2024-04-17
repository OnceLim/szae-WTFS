#ifndef POINTMASS_H
#define POINTMASS_H

#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "CGL/vector3D.h"
#include "misc/sphere_drawing.h"

using namespace CGL;

// Forward declarations
class Halfedge;
static float GAS_K = 0.1f;
static float DENSITY_OFFSET = 100;

struct PointMass {
  PointMass(Vector3D position, Vector3D color)
      : color(color), start_position(position), position(position),
        last_position(position) {}

  Vector3D normal();
  Vector3D velocity(double delta_t) {
    return (position - last_position) / delta_t;
  }

  void updatePressure() {
      this->pressure = GAS_K * (this->density - DENSITY_OFFSET);
  }

  Vector3D start_position;

  // dynamic values
  Vector3D position;
  Vector3D last_position;
  Vector3D forces;
  Vector3D color;
  double density;
  double pressure;
  double mass;
  Vector3D last_velocity;

  // mesh reference
  Halfedge *halfedge;

  // Draw pointmasses as spheres
  Misc::SphereMesh m_sphere_mesh;
};

#endif /* POINTMASS_H */
