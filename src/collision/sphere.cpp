#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(PointMass &pm, double delta_t) {
  // TODO (Part 3): Handle collisions with spheres.
    Vector3D dir = pm.position-origin;
    
    if (dir.norm() > radius) {
        return;
    }

    Vector3D normal = dir.unit();  // Normal vector from origin to the point mass.
    Vector3D tangentPoint = origin + normal * radius;  // Calculate tangent point on the sphere surface.

    Vector3D correction = tangentPoint - pm.position;  // Correction to move to the sphere's surface.
    Vector3D velocity = (pm.position - pm.last_position) / delta_t;  // Calculate current velocity of the point mass

    // Decompose the velocity into normal and tangential components
    double velocityNormalComponent = dot(velocity, normal);  // Normal component of velocity (scalar)
    Vector3D velocityNormal = normal * velocityNormalComponent;  // Normal component of velocity (vector)
    Vector3D velocityTangent = velocity - velocityNormal;  // Tangential component of velocity

    // Reflect the normal component for the bounce (perfectly elastic collision)
    // Coefficient of Restitution (cr) is typically between 0 and 1, with 1 being a perfectly elastic collision.
    double cr = 1.0; // Assuming a perfectly elastic collision
    Vector3D reflectedVelocityNormal = velocityNormal * (-cr);

    // Apply the correction to the point mass's position
    pm.position = tangentPoint + reflectedVelocityNormal * delta_t + velocityTangent * delta_t * (1 - friction);
    pm.last_position = tangentPoint;  // Update the last position to the tangent point post-collision

// 3rd iteration
//    Vector3D normal = dir.unit();  // Normal vector from origin to the point mass.
//    Vector3D tangentPoint = origin + normal * radius;  // Calculate tangent point on the sphere surface.
//
//    Vector3D correction = tangentPoint - pm.position;  // Correction to move to the sphere's surface.
//    Vector3D velocity = (pm.position - pm.last_position) / delta_t;  // Calculate current velocity of the point mass
//
//    // Decompose the velocity into normal and tangential components
//    Vector3D velocityNormal = dot(velocity, normal);  // Normal component of velocity
//    Vector3D velocityTangent = velocity - velocityNormal;  // Tangential component of velocity
//
//    // Apply the correction to the point mass's position
//    // Reflect the normal component for the bounce and apply friction to the tangential component
//    Vector3D reflectedVelocityNormal = -velocityNormal;
//    Vector3D newVelocityTangent = velocityTangent * (1 - friction);
//
//    // Update the position using the new velocity and move the point mass out of the sphere
//    pm.position = tangentPoint + (reflectedVelocityNormal + newVelocityTangent) * delta_t;
//    pm.last_position = tangentPoint;  // Update the last position to the tangent point post-collision


    //    Vector3D tangent = origin + dir.unit() * radius;
//    Vector3D correction = tangent - pm.last_position;
//
//    pm.position = pm.last_position + correction * (1-friction);


//      Iteration 2
//    Vector3D normal = dir.unit();  // Normal vector from origin to the point mass.
//    Vector3D tangentPoint = origin + normal * radius;  // Calculate tangent point on the sphere surface.
//
//    Vector3D correction = tangentPoint - pm.last_position;  // Correction to move to the sphere's surface.
//    Vector3D velocity = pm.position - pm.last_position;  // Current velocity vector of the point mass.
//
//    // Decompose the velocity into normal and tangential components
//    Vector3D velocityNormal = dot(velocity, normal);  // Normal component of velocity
//    Vector3D velocityTangent = velocity - velocityNormal;  // Tangential component of velocity
//
//    // Apply the correction to the point mass's position
//    // Here we use friction to simulate energy loss in the normal direction
//    pm.position = pm.last_position + correction + velocityTangent * (1 - friction);
}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  m_sphere_mesh.draw_sphere(shader, origin, radius * 0.92);
}
