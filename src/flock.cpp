#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "flock.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Flock::Flock(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildFlockMesh();
}

Flock::~Flock() {
  point_masses.clear();
  springs.clear();

  if (flockMesh) {
    delete flockMesh;
  }
}

void Flock::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.

    double x, y, z;
 
    for (int j = 0; j < num_height_points; j++){
        for (int i = 0; i < num_width_points; i++){
            x = i * (width / (double) num_width_points);
            Vector3D pos;
            if (orientation == HORIZONTAL) {

                y = 1;
                z = j * (height / (double)num_height_points);

            }
            else {
                y = j * (height / (double)num_height_points);
                z = (double)(rand() - (0.5 * RAND_MAX)) / RAND_MAX / 500.0;

            }


            pos = Vector3D(x, y, z);
            point_masses.emplace_back(pos, false);
        }
    }

    for (int i = 0; i < pinned.size(); i++) {
        x = pinned[i][1];
        y = pinned[i][0];
        (&point_masses[x * num_width_points + y])->pinned = true;
    }


    for (int h = 0; h < num_height_points; h++) {
        for (int w = 0; w < num_width_points; w++) {
            PointMass* pm = &point_masses[h * num_width_points + w];
            if (h != num_height_points - 1) {
                springs.emplace_back(pm, &point_masses[(h + 1) * num_width_points + w], STRUCTURAL);
            }
            if (w != num_width_points - 1) {
                springs.emplace_back(pm, &point_masses[h * num_width_points + w + 1], STRUCTURAL);
            }


            if (h != num_height_points - 1 && w != num_width_points - 1) {
                springs.emplace_back(pm, &point_masses[(h + 1) * num_width_points + w + 1], SHEARING);
            }
            if (h != num_height_points - 1 && w != 0) {
                springs.emplace_back(pm, &point_masses[(h + 1) * num_width_points + w - 1], SHEARING);
            }


            if (h < num_height_points - 2) {
                springs.emplace_back(pm, &point_masses[(h + 2) * num_width_points + w], BENDING);
            }
            if (w < num_width_points - 2) {
                springs.emplace_back(pm, &point_masses[h * num_width_points + w + 2], BENDING);
            }
        }
    }



}

void Flock::simulate(double frames_per_sec, double simulation_steps, FlockParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  double mass = 10.0;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.
  Vector3D acceleration = Vector3D();
  for (int i = 0; i < external_accelerations.size(); i++) {
      acceleration += external_accelerations[i];
  }
  for (int i = 0; i < point_masses.size(); i++) {
      PointMass* p = &point_masses[i];
      p->forces = Vector3D();
      p->forces += mass * acceleration;
  }


  // TODO (Part 2): Use Verlet integration to compute new point mass positions

  // TODO (Part 4): Handle self-collisions.
  build_spatial_map();
  for (int i = 0; i < point_masses.size(); i++) {
      PointMass* p = &point_masses[i];
      if (p->pinned) {
          continue;
      }
      self_collide(*p, simulation_steps);
  }

  // TODO (Part 3): Handle collisions with other primitives.
  for (int i = 0; i < point_masses.size(); i++) {
      PointMass* p = &point_masses[i];
      if (p->pinned) {
          continue;
      }
      for (int j = 0; j < (*collision_objects).size(); j++) {
          (*(*collision_objects)[j]).collide(*p);
      }
  }

  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
  for (int i = 0; i < springs.size(); i++) {
      Spring* sp = &springs[i];

      double over = (sp->pm_a->position - sp->pm_b->position).norm() - (sp->rest_length * 1.1);
      if (over > 0.0) {

          if (sp->pm_a->pinned && sp->pm_b->pinned) {
              continue;
          }
          else if (sp->pm_a->pinned) {
              sp->pm_b->position += (sp->pm_a->position - sp->pm_b->position).unit() * over;
          }
          else if (sp->pm_b->pinned) {
              sp->pm_a->position += (sp->pm_b->position - sp->pm_a->position).unit() * over;
          }
          else {
              sp->pm_a->position += (sp->pm_b->position - sp->pm_a->position).unit() * over * 0.5;
              sp->pm_b->position += (sp->pm_a->position - sp->pm_b->position).unit() * over * 0.5;
          }
      }
  }
}

void Flock::build_spatial_map() {
    
  for (const auto &entry : map) {
      
    delete(entry.second);
  }
  map.clear();
  
  for (int i = 0; i < point_masses.size(); i++) {
      PointMass* p = &point_masses[i];
      float hash = hash_position(p->position);
      
      if (map.find(hash) == map.end()) {
          
          map.emplace(hash,  new vector<PointMass*> ());
          
          map[hash]->push_back(p);
          
      }
      else {
          
          map[hash]->push_back(p);
          
      }

      
  }

}

void Flock::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.
    float hash = hash_position(pm.position);
    Vector3D sum;
    double count = 0.0;
    for (PointMass* candidate : *map[hash]) {
       
       if (candidate == &pm) {
           continue;
       }
       if ((candidate->position - pm.position).norm() < 2 * thickness) {
           count += 1.0;
           sum += (2*thickness - (candidate->position - pm.position).norm())* (pm.position - candidate->position).unit();
       }
    }
    if (count == 0.0) {
        return;
    }
    sum = sum / count / simulation_steps;
    pm.position += sum; 
    return; 
}

float Flock::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
    
    float w = 3 * width / num_width_points;
    float h = 3 * height / num_height_points;
    float t = max(w, h);
    
    int bx = floor(pos.x / w);
    int by = floor(pos.y / h);
    int bz = floor(pos.z / t);
    
    float hash = (float)bx;
    hash = hash * pow(2, 16);
    hash += by;
    hash *= pow(2, 8);
    hash += bz;
    
  return hash;
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Flock::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Flock::buildFlockMesh() {
  if (point_masses.size() == 0) return;

  FlockMesh *flockMesh = new FlockMesh();
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

  // Go back through the flock mesh and link triangles together using halfedge
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

  flockMesh->triangles = triangles;
  this->flockMesh = flockMesh;
}
