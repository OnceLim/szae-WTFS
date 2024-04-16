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

bool checkPinned(Vector3D pos, vector<vector<int>> pinned) {
    if (!(pos.x >= pinned[0][0] && pos.x <= pinned[1][0])) {
        return false;
    } else if (!(pos.y >= pinned[0][1] && pos.y <= pinned[1][1])) {
        return false;
    }
    return true;
}

int getPMAbove(Vector3D pos) {
    
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.
    
    if (num_width_points == 0 || num_height_points == 0) {
        return;
    }

    double width_diff = width/(double) (num_width_points - 1);
    double height_diff = height/(double) (num_height_points - 1);
    double width_start = width_diff/2;
    double height_start = height_diff/2;

    //Create an evenly spaced grid of masses
    if (orientation == HORIZONTAL) {
        for (int i=0; i<num_height_points; ++i) {
            for (int j=0; j<num_width_points; ++j) {
                Vector3D pos = Vector3D(width_start+j*width_diff, 1, height_start+i*height_diff);                
                PointMass pm = PointMass(pos, Vector3D(0, 0, 0));
                point_masses.emplace_back(pm);
            }
        }
    } else if (orientation == VERTICAL) {
        double min_off = -1.0 / 1000;
        double max_off = 1.0 / 1000;
        double y = 0;
        for (int j = 0; j < num_height_points; ++j) {
            double x = 0;
            for (int i = 0; i < num_width_points; ++i) {
                double rand_off = (double) rand() / RAND_MAX - 1;
                double z = min_off + rand_off * (max_off - min_off);
                point_masses.emplace_back(Vector3D(x, y, z), false);
                x += width_diff;
            }
            y += height_diff;
        }
    }

    // for (vector<int> coordo : pinned) {
    //     std::cout <<"Hello"<< std::endl;
    //     point_masses[coordo[1] * num_width_points + coordo[0]].pinned = true;
    // }


    //Create springs
    /*
    for (int i=0; i<num_height_points; ++i) {
        for (int j=0; j<num_width_points; ++j) {
            int pm_index = i * num_width_points + j;
            int pm_index_left = pm_index - 1;
            int pm_index_up = (i-1) * num_width_points + j;
            int pm_index_diag_up_left = (i-1) * num_width_points + (j-1);
            int pm_index_diag_up_right = (i-1) * num_width_points + (j+1);
            int pm_two_left = pm_index - 2;
            int pm_two_above = (i-2) * num_width_points + j;

            bool edge_up = (i==0);
            bool edge_left = (j==0);
            bool edge_down = (i==num_height_points-1);
            bool edge_right = (j==num_width_points-1);
            bool edge_left_2 = (j==0 || j == 1);
            bool edge_up_2 = (i==0 || i == 1);

            if (!edge_left) {
                springs.emplace_back(Spring(&point_masses[pm_index], &point_masses[pm_index_left], STRUCTURAL));
            }
            if (!edge_up) {
                springs.emplace_back(Spring(&point_masses[pm_index], &point_masses[pm_index_up], STRUCTURAL));
            }
            if (!edge_left && !edge_up) {
                springs.emplace_back(Spring(&point_masses[pm_index], &point_masses[pm_index_diag_up_left], SHEARING));
            }
            if (!edge_right && !edge_up) {
                springs.emplace_back(Spring(&point_masses[pm_index], &point_masses[pm_index_diag_up_right], SHEARING));
            }
            if (!edge_left_2) {
                springs.emplace_back(Spring(&point_masses[pm_index], &point_masses[pm_two_left], BENDING));
            }
            if (!edge_up_2) {
                springs.emplace_back(Spring(&point_masses[pm_index], &point_masses[pm_two_above], BENDING));
            }
        }
    }*/

}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.
  Vector3D total_external_forces;
  for(const Vector3D& acc: external_accelerations)
      total_external_forces += acc;

  total_external_forces *= mass;
  for(PointMass &pm: this->point_masses)
      pm.forces = total_external_forces;

  /*
  for (Spring &s : springs) {
    if (cp->enable_structural_constraints || cp->enable_shearing_constraints || cp->enable_bending_constraints) {
        double scalar = cp->ks * ((s.pm_a->position - s.pm_b->position).norm() - s.rest_length);
        if (s.spring_type == BENDING)
            scalar *= 0.2;

        Vector3D f = scalar * ((s.pm_b->position - s.pm_a->position).unit());
        s.pm_a->forces += f;
        s.pm_b->forces -= f;
    }
  }*/

  // TODO (Part 2): Use Verlet integration to compute new point mass positions

  for (PointMass &pm : point_masses) {
    // if (!pm.pinned) {
        Vector3D curr_pos = pm.position;
        Vector3D last_pos = pm.last_position;
        Vector3D accel = pm.forces / mass;
        pm.position = curr_pos + (double)(1.0f - cp->damping / 100.0f) * (curr_pos - last_pos) + accel * pow(delta_t, 2);
        pm.last_position = curr_pos;
    // }
  }


  // TODO (Part 4): Handle self-collisions.
  build_spatial_map();
  for (PointMass &pm : point_masses) {
      self_collide(pm, simulation_steps);
  }


//   TODO (Part 3): Handle collisions with other primitives.
    for (PointMass &pm : point_masses) {
        for (CollisionObject *co : *collision_objects) {
            co->collide(pm, delta_t);
        }
    }

  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
/*
  for (Spring s : springs) {
      if (cp->enable_bending_constraints || cp->enable_shearing_constraints || cp->enable_structural_constraints) {
          // Check if point masses are pinned
          if (s.pm_a->pinned && s.pm_b->pinned)
            continue;
          // Check if spring's length less than 110% of its rest length
          double diff = (s.pm_b->position - s.pm_a->position).norm() - (double) 1.1 * s.rest_length;
          if (diff <= 0)
              continue;
          Vector3D correction = diff * (s.pm_b->position - s.pm_a->position).unit();
          if (s.pm_a->pinned) {
              s.pm_b->position -= correction;
          }
          else if (s.pm_b->pinned) {
              s.pm_a->position += correction;
          }
          else {
              correction *= 0.5;
              s.pm_b->position -= correction;
              s.pm_a->position += correction;
          }
      }
  }*/

}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
  for (PointMass &pm : point_masses) {
      float hash = hash_position(pm.position);
      if (!map[hash]) {
          map[hash] = new vector<PointMass*>;
      }
      map[hash]->push_back(&pm);
  }

}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.

    float hash = hash_position(pm.position);
    Vector3D correction;
    int count = 0;
    if(map[hash])
    {
        for(PointMass *pm_iter: *map[hash])
        {
            Vector3D diff = pm.position - pm_iter->position;
            if(pm_iter!=&pm && diff.norm() <= 1 * thickness)
            {
                correction += diff.unit() * (1 * thickness - diff.norm());
                count++;
            }
        }
    }
    if(count)
        pm.position += correction / count / simulation_steps;

}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
  double w = 3.0 * width / (num_width_points - 1);
  double h = 3.0 * height / (num_height_points - 1);
  double t = max(w, h);

  float x = floor(pos.x / w);
  float y = floor(pos.y / h);
  float z = floor(pos.z / t);

  float hash = (x * 31 + y) * 31 + z;

  return hash;

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
