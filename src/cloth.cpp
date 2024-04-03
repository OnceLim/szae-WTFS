#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.
  double width_interval = width / num_width_points;
  double height_interval = height / num_height_points;
  for (int i = 0; i < num_height_points; i++) {
    for (int j = 0; j < num_width_points; j++) {
      bool pinnedCheck = false;
      //check if within vector
      for (vector<int>& v: pinned) {
        if (v[0] == j && v[1] == i) {
          pinnedCheck = true;
        }
      }
      if (orientation == HORIZONTAL) {
        PointMass newPointMass = PointMass(Vector3D(j * width_interval, 1.0f, i * height_interval), pinnedCheck);
        point_masses.emplace_back(newPointMass);
      } else {
        double z_interval = rand() / double(RAND_MAX) * 0.002 - 0.001;
        PointMass newPointMass = PointMass(Vector3D(j * width_interval, i * height_interval, z_interval), pinnedCheck);
        point_masses.emplace_back(newPointMass);
      }
    }
  }
  for (int i = 0; i < num_width_points; ++i) {
    for (int j = 0; j < num_height_points; ++j) {
      PointMass* curr = &point_masses[i + num_width_points * j];
      if (i > 0) {
        springs.emplace_back(curr, curr - 1, STRUCTURAL);
      }
      if (j > 0) {
        springs.emplace_back(curr, curr - num_width_points, STRUCTURAL);
      }
      if (i > 0 && j > 0) {
        springs.emplace_back(curr, curr - num_width_points - 1, SHEARING);
      }
      if (i < num_width_points - 1 && j > 0) {
        springs.emplace_back(curr, curr - num_width_points + 1, SHEARING);
      }
      if (i > 1) {
        springs.emplace_back(curr, curr - 2, BENDING);
      }
      if (j > 1) {
        springs.emplace_back(curr, curr - 2 * num_width_points, BENDING);
      }
    }
  }
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.
  for (PointMass& point: point_masses) {
    point.forces = Vector3D(0, 0, 0);
    for (Vector3D& acceleration: external_accelerations) {
      point.forces += acceleration * mass;
    }
  }

  for (Spring& spring : springs) {
    if ((spring.spring_type == STRUCTURAL && cp->enable_structural_constraints) ||
        (spring.spring_type == SHEARING && cp->enable_shearing_constraints) ||
        (spring.spring_type == BENDING && cp->enable_bending_constraints)) {
      double springconstant;
      if (spring.spring_type == BENDING) {
          springconstant = cp->ks * 0.2;
      } else {
          springconstant = cp->ks;
      }
      PointMass* pointA = spring.pm_a;
      PointMass* pointB = spring.pm_b;
      Vector3D vectorAB = pointB->position - pointA->position;
      double distance = vectorAB.norm();
      Vector3D forceDirection = vectorAB / distance;
      double forceMagnitude = springconstant * (distance - spring.rest_length);
      Vector3D force = forceDirection * forceMagnitude;
      pointA->forces += force;
      pointB->forces -= force;
    }
  }

  // TODO (Part 2): Use Verlet integration to compute new point mass positions
  for (PointMass& point: point_masses) {
    if (!point.pinned) {
      Vector3D newposition = point.position + (1.0 - cp->damping / 100) * (point.position - point.last_position) + (point.forces / mass) * delta_t * delta_t;
      point.last_position = point.position;
      point.position = newposition;
    }
  }  

  // TODO (Part 4): Handle self-collisions.
  build_spatial_map();
  for (PointMass& point: point_masses) {
      self_collide(point, simulation_steps);
  }

  // TODO (Part 3): Handle collisions with other primitives.
  for (PointMass& point : point_masses) {
    for (auto c : *collision_objects) {
      c->collide(point);
    }
  }
  

  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
  for (Spring& spring: springs) {
    if ((spring.spring_type == STRUCTURAL && cp->enable_structural_constraints) ||
        (spring.spring_type == SHEARING && cp->enable_shearing_constraints) ||
        (spring.spring_type == BENDING && cp->enable_bending_constraints)) {
      PointMass* massA = spring.pm_a;
      PointMass* massB = spring.pm_b;
      Vector3D displacement = massB->position - massA->position;
      double distance = displacement.norm();
      if (distance > spring.rest_length * 1.1) {
        displacement.normalize();
        Vector3D constraint = displacement * spring.rest_length * 1.1;
        if (!massA->pinned && !massB->pinned) {
          Vector3D midpoint = (massA->position + massB->position) * 0.5;
          massA->position = midpoint - 0.5 * constraint;
          massB->position = midpoint + 0.5 * constraint;
        } else if (!massA->pinned) {
          massA->position = massB->position - constraint;
        } else if (!massB->pinned) {
          massB->position = massA->position + constraint;
        }
      }
    }
  }
}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();
  // TODO (Part 4): Build a spatial map out of all of the point masses.
  for (PointMass& point: point_masses) {
    float hash = hash_position(point.position);
    if (map.find(hash) == map.end()) {
      map[hash] = new vector<PointMass*>;
    }
    map[hash]->push_back(&point);
  }

}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.
  Vector3D Correction(0, 0, 0);
  int count = 0;
  float hash = hash_position(pm.position);
  for (PointMass *candidate: *map[hash]) {
    if (&pm != candidate) {
      Vector3D difference = pm.position - candidate->position;
      double distance = difference.norm();
      if (2 * thickness - distance > 0) {
        difference.normalize();
        Correction += difference * (2 * thickness - distance);
        count++;
      }
    }
  }
  if (count) {
    pm.position += Correction / count / simulation_steps;
  }
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
  float w, h, t, truncatedx, truncatedy, truncatedz;
  w = 3 * width / num_width_points;
  h = 3 * height / num_height_points;
  t = max(w, h);
  truncatedx = pos.x - fmod(pos.x, w);
  truncatedy = pos.y - fmod(pos.y, h);
  truncatedz = pos.z - fmod(pos.z, t);
  return truncatedx + 1000 * truncatedy + 1000 * 1000 * truncatedz;
  return 0.f;
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
