#include <iostream>
#include <math.h>
#include <random>
#include <vector>
#include <random>

#include "flock.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Flock::Flock(double width, double height, int num_width_points,
             int num_height_points, float thickness)
{
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  buildGrid();
  buildFlockMesh();
}

Flock::~Flock()
{
  point_masses.clear();
  springs.clear();
  birds.clear();
  if (flockMesh)
  {
    delete flockMesh;
  }
}

void Flock::buildGrid()
{
  // TODO (Part 1): Build a grid of masses and springs.
  /*
  for (int j = 0; j < num_height_points; j += 1)
  {
    for (int i = 0; i < num_width_points; i += 1)
    {
      double x = (double)width * (double)i / (double)num_width_points;
      double y = (double)height * (double)j / (double)num_height_points;
      Vector3D center;
      bool pin = false;
      for (int k = 0; k < pinned.size(); k++) {
        if ((pinned[k][0] == i) && (pinned[k][1] == j)) {
          pin = true;
          break;
        }
      }
      if (orientation == 0)
      {
        center = Vector3D(x + rand() % 100 / 1000., 1., y);
        std::cout << x << " " << y << " " << center.x << endl;
      }
      else
      {
        double sign = random() > .5 ? 1. : -1.;
        double rand_offset = sign * rand() / 1000. /  2147483647.;
        center = Vector3D(x + rand(), y + rand(), rand_offset);
      }
      point_masses.emplace_back(PointMass(center, pin));
    }
  }*/
  for (int i = 0; i < num_birds; i += 1) {
    point_masses.emplace_back(PointMass(Vector3D(rand() % 100 / 100., rand() % 100 / 100., rand() % 100 / 100.), false));
  }

  PointMass *a;
  for (int i = 0; i < point_masses.size(); i++) {
    a = &point_masses[i];
    birds.emplace_back(Bird(a));
  }
}

vector<PointMass> Flock::getNeighbours(PointMass pm, double range) {
  vector<PointMass> npms;
  for (PointMass &p : point_masses) {
    double dis = (p.position - pm.position).norm();
    if (dis > 0 && dis < range) {
      PointMass temp = PointMass(p.position, false);
      temp.speed = p.speed;
      npms.push_back(temp);
    }
  }
  return npms;
}

void Flock::simulate(double frames_per_sec, double simulation_steps, FlockParameters *fp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects,
                     Vector3D windDir)
{
  // need more birds
  if (fp->num_birds > point_masses.size()) {
    int num = fp->num_birds;
    for (int i = point_masses.size(); i < num; i += 1) {
      point_masses.emplace_back(PointMass(Vector3D(rand() % 100 / 100., rand() % 100 / 100., rand() % 100 / 100.), false));
      // a = &point_masses[i];
      // birds.emplace_back(Bird(a));
    }
    PointMass *a;
    for (int i = point_masses.size(); i < num; i += 1) {
      a = &point_masses[i];
      birds.emplace_back(Bird(a));
    }
  } else if (fp->num_birds < point_masses.size()) { // remove extra birds
    int extra = point_masses.size() - fp->num_birds;
    for (int i = 0; i < extra; i += 1) { 
      point_masses.pop_back();
      birds.pop_back();
    }
  }
  // cohesion
  for (PointMass &point_mass: point_masses) {
    Vector3D goal = Vector3D();
    vector<PointMass> neighbour = getNeighbours(point_mass, fp->coherence);
    for (PointMass &npm : neighbour) {
      goal = goal + npm.position;
    }
    goal = goal / neighbour.size();
    point_mass.cumulatedSpeed += goal;
    if (neighbour.size() == 1) {
      point_mass.cumulatedSpeed = Vector3D();
    }
  }
  // separation 
  for (PointMass &point_mass: point_masses) {
    Vector3D goal = Vector3D();
    vector<PointMass> neighbour = getNeighbours(point_mass, fp->separation);
    for (PointMass &npm : neighbour) {
      goal = goal + point_mass.position - npm.position;
    }
    goal = goal / neighbour.size();
    if (neighbour.size() > 1) {
      point_mass.cumulatedSpeed += goal;
    }
  }
  // alignment 
  for (PointMass &point_mass: point_masses) {
    Vector3D speed = 0.;
    double velocity = 0.;
    vector<PointMass> neighbour = getNeighbours(point_mass, fp->alignment);
    for (PointMass &npm : neighbour) {
      speed += npm.speed - point_mass.speed;
    }
    speed = speed / neighbour.size() / 100000.;
    point_mass.cumulatedSpeed += speed;
  }

  for (PointMass &point_mass: point_masses) {
    point_mass.speed += point_mass.cumulatedSpeed * .0000001;
    point_mass.cumulatedSpeed = 0;
    Vector3D dir = point_mass.speed;
    dir.normalize(); 
    point_mass.speed = dir * max(min(point_mass.speed.norm(), point_mass.maxSpeed), -point_mass.maxSpeed);
    if (point_mass.position.x > x  || point_mass.position.x < 0 ) { // random bounce to -random, random, random
      point_mass.speed.x *= -1;
    }
    if (point_mass.position.y > y || point_mass.position.y < 0) { // random bounce to random, -random, random
      point_mass.speed.y *= -1;
    }
    if (point_mass.position.z > z || point_mass.position.z < 0 ) { // random bounce to random, random, -random
      point_mass.speed.z *= -1;
    }
    point_mass.position += point_mass.speed + windDir * .0000001;
      // std::cout << isnan(point_mass.position.x) << endl;
    if (isnan(point_mass.position.x)) {
      point_mass = PointMass(Vector3D(rand() % 100 / 100., rand() % 100 / 100., rand() % 100 / 100.), false);
      std::cout << point_mass.position << " " << point_mass.speed << endl;
    }
  }

  /*
  double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;

  // TODO (Part 2): Compute total force acting on each point mass.
  // external force to every point mass
  Vector3D total_acceleration = Vector3D();
  for (Vector3D acceleration: external_accelerations) {
    total_acceleration += acceleration;
  }
  Vector3D force = total_acceleration * mass;
  for (PointMass &point_mass: point_masses) {
    point_mass.forces = Vector3D(force);
  }
  //apply the spring correction forces
  for (Spring &spring: springs) {
    if ((spring.spring_type==2 && !cp->enable_bending_constraints) ||
        (spring.spring_type==1 && !cp->enable_shearing_constraints) ||
        (spring.spring_type==0 && !cp->enable_structural_constraints)) {
      continue;
    } else {
      Vector3D diff = spring.pm_a->position - spring.pm_b->position;
      double correction_force_mag = (diff.norm() - spring.rest_length) * cp->ks;
      if (spring.spring_type==2) {
        correction_force_mag *= 0.2;
      }
      spring.pm_a->forces += (spring.pm_b->position - spring.pm_a->position).unit() * correction_force_mag;
      spring.pm_b->forces += (spring.pm_a->position - spring.pm_b->position).unit() * correction_force_mag;
    }
  }
    
  // TODO (Part 2): Use Verlet integration to compute new point mass positions
  for (PointMass &point_mass: point_masses) {
    if (!point_mass.pinned) {
      Vector3D at = point_mass.forces / mass;
      Vector3D new_position = point_mass.position + (1 - cp->damping/100.0) * (point_mass.position - point_mass.last_position) + at * delta_t * delta_t;
      point_mass.last_position = Vector3D(point_mass.position);
      point_mass.position = new_position;
    }
  }
  
  // TODO (Part 4): Handle self-collisions.
  build_spatial_map();
  for (PointMass &point_mass : point_masses) {
    self_collide(point_mass, simulation_steps);
  }

  // TODO (Part 3): Handle collisions with other primitives.
  for (PointMass &point_mass : point_masses) {
    for (CollisionObject *collision_object : *collision_objects) {
      collision_object->collide(point_mass);
    }
  }

  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
  for (Spring &spring: springs) {
    double distance = (spring.pm_a->position - spring.pm_b->position).norm();
    if (spring.rest_length*1.1 < distance) {
      if (spring.pm_a->pinned && spring.pm_b->pinned) {
        continue;
      } else if (spring.pm_a->pinned) {
        spring.pm_b->position = (spring.pm_b->position - spring.pm_a->position).unit() * 1.1 * spring.rest_length + spring.pm_a->position;
      } else if (spring.pm_b->pinned) {
        spring.pm_a->position = (spring.pm_a->position - spring.pm_b->position).unit() * 1.1 * spring.rest_length + spring.pm_b->position;
      } else {
        Vector3D midpoint = (spring.pm_a->position + spring.pm_b->position) / 2.0;
        spring.pm_a->position = (spring.pm_a->position - spring.pm_b->position).unit() * 0.55 * spring.rest_length + midpoint;
        spring.pm_b->position = (spring.pm_b->position - spring.pm_a->position).unit() * 0.55 * spring.rest_length + midpoint;
      }
    }
  }*/
}

void Flock::build_spatial_map()
{
  for (const auto &entry : map)
  {
    delete (entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
  for (PointMass &point_mass : point_masses) {
    float hashed = hash_position(point_mass.position);
    if (!map[hashed]) {
      map[hashed] = new vector<PointMass *> ();
    }
    map[hashed]->push_back(&point_mass);
  }
}

void Flock::self_collide(PointMass &pm, double simulation_steps)
{
  // TODO (Part 4): Handle self-collision for a given point mass.
  float hashed = hash_position(pm.position);
  Vector3D temp;
  int i = 0;
  if (map[hashed]) { 
    for (PointMass *point_mass : *map[hashed]) {
      Vector3D dis = pm.position - point_mass->position;
      if (point_mass != &pm && dis.norm() <= 2 * thickness) {
        temp += (2 * thickness - dis.norm()) * dis.unit();
        i += 1;
      }
    }
  }
  if (i) {
    pm.position += temp / (i * simulation_steps);
  }
}

float Flock::hash_position(Vector3D pos)
{
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
  float w = 3 * width / num_width_points,
        h = 3 * height / num_height_points,
        t = max(w, h);
  float x = floor(pos[0] / w),
        y = floor(pos[1] / h),
        z = floor(pos[2] / t);
  // return fmod(x + y * 31 + z * 31 * 31, num_width_points * num_height_points);// need mod?
  return x + y * 233. + z * 233. * 233.;// need mod?
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////


void Flock::follow(double x, double y) {
    for (PointMass p : point_masses) {
        p.cumulatedSpeed += p.position - Vector3D(x, y, 0);
    }
}

void Flock::reset()
{
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++)
  {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Flock::buildFlockMesh()
{
  if (point_masses.size() == 0)
    return;

  FlockMesh *flockMesh = new FlockMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++)
  {
    for (int x = 0; x < num_width_points - 1; x++)
    {
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

      PointMass *pm_A = pm;
      PointMass *pm_B = pm + 1;
      PointMass *pm_C = pm + num_width_points;
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
  for (int i = 0; i < triangles.size(); i++)
  {
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
  for (int i = 0; i < triangles.size(); i++)
  {
    Triangle *t = triangles[i];

    if (topLeft)
    {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0)
      { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      }
      else
      {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris)
      { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      }
      else
      {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    }
    else
    {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1)
      { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      }
      else
      {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f)
      { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      }
      else
      {
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
