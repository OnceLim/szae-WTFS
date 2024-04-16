#include "iostream"
#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../clothSimulator.h"
#include "plane.h"

using namespace std;
using namespace CGL;

#define SURFACE_OFFSET 0.0001

//double get_t(Vector3D pos, Vector3D normal, Vector3D point) {
//    return dot(point - pos, normal) / dot(normal, normal);
//}

void Plane::collide(PointMass &pm, double delta_t) {
  // TODO (Part 3): Handle collisions with planes.
    
    if (dot(point - pm.position, normal) * dot(point - pm.last_position, normal) > 0.0) {
        return;
    } 
//    else if (dot(point - pm.position, normal) == 0.0 && dot(point - pm.last_position, normal) != 0.0) {
//        return;
//    } else if (dot(point - pm.position, normal) != 0.0 && dot(point - pm.last_position, normal) == 0.0) {
//        return;
//    }
    
    double t = dot(point - pm.position, normal) / dot(normal, normal);
    
    Vector3D tangent = pm.position + t*normal;
    
    Vector3D correction = tangent - pm.last_position + normal * SURFACE_OFFSET;

    pm.position = pm.last_position + correction * (1.0-friction);

}

void Plane::render(GLShader &shader) {
  nanogui::Color color(0.7f, 0.7f, 0.7f, 1.0f);

  Vector3f sPoint(point.x, point.y, point.z);
  Vector3f sNormal(normal.x, normal.y, normal.z);
  Vector3f sParallel(normal.y - normal.z, normal.z - normal.x,
                     normal.x - normal.y);
  sParallel.normalize();
  Vector3f sCross = sNormal.cross(sParallel);

  MatrixXf positions(3, 4);
  MatrixXf normals(3, 4);

  positions.col(0) << sPoint + 2 * (sCross + sParallel);
  positions.col(1) << sPoint + 2 * (sCross - sParallel);
  positions.col(2) << sPoint + 2 * (-sCross + sParallel);
  positions.col(3) << sPoint + 2 * (-sCross - sParallel);

  normals.col(0) << sNormal;
  normals.col(1) << sNormal;
  normals.col(2) << sNormal;
  normals.col(3) << sNormal;

  if (shader.uniform("u_color", false) != -1) {
    shader.setUniform("u_color", color);
  }
  shader.uploadAttrib("in_position", positions);
  if (shader.attrib("in_normal", false) != -1) {
    shader.uploadAttrib("in_normal", normals);
  }

  shader.drawArray(GL_TRIANGLE_STRIP, 0, 4);
}
