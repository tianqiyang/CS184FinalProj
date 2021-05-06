#include <iostream>
#include <math.h>
#include <random>
#include <vector>
#include <random>
#include <stdlib.h>
#include "flock.h"
#include "collision/plane.h"
#include "collision/sphere.h"
#include "collision/cylinder.h"

using namespace std;

double STOP_RATE = 0.8;
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

Vector3D Flock::generatePos()
{
  double x, y, z;
  x = (double)(rand() % 100) / 100.;
  y = (double)(rand() % 100) / 100.;
  z = (double)(rand() % 100) / 100.;
  return Vector3D(x, y, z);
}

void initializeSpeed(PointMass *pm)
{
  double sx, sy, sz;
  sx = -pm->maxSpeed + rand() / (RAND_MAX / (pm->maxSpeed + pm->maxSpeed));
  sy = -pm->maxSpeed + rand() / (RAND_MAX / (pm->maxSpeed + pm->maxSpeed));
  sz = -pm->maxSpeed + rand() / (RAND_MAX / (pm->maxSpeed + pm->maxSpeed));
  pm->speed = Vector3D(sx, sy, sz);
  Vector3D dir = pm->speed;
  dir.normalize();
  pm->speed = dir * CGL::clamp(pm->speed.norm(), pm->minSpeed, pm->maxSpeed);
}

void Flock::buildGrid()
{

  for (int i = 0; i < num_birds; i += 1)
  {
    PointMass pm = PointMass(generatePos(), false);
    pm.able_stop = (rand() % 10 / 10. < STOP_RATE);
    initializeSpeed(&pm);
    point_masses.emplace_back(pm);
  }

  PointMass *a;
  for (int i = 0; i < point_masses.size(); i++)
  {
    a = &point_masses[i];
    point_masses[i].rand_stop_pos = NULL;
    birds.emplace_back(Bird(a));
  }
}

vector<PointMass> Flock::getNeighbours(PointMass pm, double range)
{
  vector<PointMass> npms;
  for (PointMass &p : point_masses)
  {
    double dis = (p.position - pm.position).norm();
    if (dis < range && &p != &pm)
    {
      PointMass temp = PointMass(p.position, false);
      temp.speed = p.speed;
      npms.push_back(temp);
    }
  }
  return npms;
}

vector<vector<PointMass *> > Flock::getNeighbours(PointMass pm, vector<double> range)
{
  vector<PointMass *> n1, n2, n3;
  vector<vector<PointMass *> > vecs = vector<vector<PointMass *> >({n1, n2, n3});
  for (PointMass &p : point_masses)
  {
    double dis = (p.position - pm.position).norm();
    for (int i = 0; i < 3; i++)
    {
      if (dis < range[i] && &p != &pm)
      {
        vecs[i].push_back(&p);
      }
    }
  }
  return vecs;
}
Vector3D normalizeForce(Vector3D acceleration)
{
  acceleration.normalize();
  double clampAcc = CGL::clamp(acceleration.norm(), 0.0, .000001);

  return acceleration * clampAcc;
}

void Flock::simulate(double frames_per_sec, double simulation_steps, FlockParameters *fp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects,
                     Vector3D windDir, bool is_stopped)
{
  // need more birds
  Cylinder *cylinder = dynamic_cast<Cylinder *>(collision_objects->at(0));
  vector<vector<Vector3f> > stopLine = cylinder->stopLine;
  if (fp->num_birds > point_masses.size())
  {
    // only add one bird at one frame
    point_masses.emplace_back(PointMass(generatePos(), false));
    PointMass *a;
    a = &point_masses[point_masses.size()-1];
    birds.emplace_back(Bird(a));
  }
  else if (fp->num_birds < point_masses.size())
  { // remove extra birds
    int extra = point_masses.size() - fp->num_birds;
    for (int i = 0; i < extra; i += 1)
    {
      point_masses.pop_back();
      birds.pop_back();
    }
  }

  double cw, sw, aw, dw;
  double dweight = 5.;
  double sum = coherence_weight + separation_weight + alignment_weight + dweight;
  cw = coherence_weight / sum;
  sw = separation_weight / sum;
  aw = alignment_weight / sum;
  dw = dweight / sum;
  for (PointMass &point_mass : point_masses)
  {
    // if "S" is not pressed or bird is not within 0.5 distance from bar, not affected by
    // stopping behavior
    if (point_mass.branch == -1) {
      point_mass.branch = rand() % cylinder->branchNum;
    }
    vector<Vector3f> line = stopLine[point_mass.branch];
    Vector3D a(line[0][0], line[0][1], line[0][2]);
    Vector3D b(line[1][0], line[1][1], line[1][2]);
    double dis = cylinder->computeDistance(point_mass.position, a, b);
    if (!is_stopped || dis > 0.5) // || !(point_mass.position[1] > a[1] && point_mass.position[1] > b[1]))
    {
      vector<double> pars = vector<double>({fp->coherence, fp->separation, fp->alignment});
      vector<vector<PointMass *> > vecs = getNeighbours(point_mass, pars);
      Vector3D goal = Vector3D();
      if (following)
      {
        double newCenterX = cursor.position.x * point_masses.size() - point_mass.position.x;
        double newCenterY = cursor.position.y * point_masses.size() - point_mass.position.y;
        double newCenterZ = cursor.position.z * point_masses.size() - point_mass.position.z;
        Vector3D newCenter = Vector3D(newCenterX, newCenterY, newCenterZ) / (point_masses.size() - 1);
        Vector3D goal = (newCenter - point_mass.position);
        point_mass.cumulatedSpeed += goal * cw;
      }
      else
      {
        for (PointMass *npm : vecs[0])
        {
          goal = goal + npm->position;
        }
        if (vecs[0].size() != 0)
        {
          goal = goal / vecs[0].size();
        }
        point_mass.cumulatedSpeed += (goal - point_mass.position) * cw;
      }
      goal = Vector3D();
      for (PointMass *npm : vecs[1])
      {
        Vector3D sep = (point_mass.position - npm->position);
        goal += sep;
      }
      if (vecs[1].size() != 0)
      {
        goal = goal / vecs[1].size();
      }
      point_mass.cumulatedSpeed += goal * sw;
      goal = Vector3D();
      for (PointMass *npm : vecs[2])
      {
        goal = goal + npm->speed;
      }
      point_mass.cumulatedSpeed += goal * aw;
    }
  }

  for (PointMass &point_mass : point_masses)
  {
    // if "S" is not pressed or bird is not within 0.5 distance from bar, not affected by
    // stopping behavior
    vector<Vector3f> line = stopLine[point_mass.branch];
    Vector3D a(line[0][0], line[0][1], line[0][2]);
    Vector3D b(line[1][0], line[1][1], line[1][2]);
    double dis = cylinder->computeDistance(point_mass.position, a, b);
    if (!is_stopped || dis > 0.5)//  || !(point_mass.position[1] > a[1] && point_mass.position[1] > b[1]))
    {
      /*Vector3D dir = point_mass.speed;
        dir.normalize();
        point_mass.speed = dir * max(min(point_mass.speed.norm(), point_mass.maxSpeed), -point_mass.maxSpeed);*/
      Vector3D decceleration = Vector3D(0, 0, 0);

      if (point_mass.position.x > x || point_mass.position.x < -x)
      {
        decceleration += (Vector3D(0.5, 0.5, 0.5) - point_mass.position) * abs(point_mass.position.x - x);
      }
      if (point_mass.position.y > y || point_mass.position.y < 0)
      {
        decceleration += (Vector3D(0.5, 0.5, 0.5) - point_mass.position) * abs(point_mass.position.y - y);
      }
      if (point_mass.position.z > z || point_mass.position.z < -z)
      {
        decceleration += (Vector3D(0.5, 0.5, 0.5) - point_mass.position) * abs(point_mass.position.z - z);
      }

      point_mass.cumulatedSpeed += (decceleration)*dw;

      Vector3D accDir = point_mass.cumulatedSpeed;
      accDir.normalize();
      point_mass.cumulatedSpeed = accDir * CGL::clamp(point_mass.cumulatedSpeed.norm(), point_mass.minAcc, point_mass.maxAcc) * 0.00001;

      point_mass.speed += point_mass.cumulatedSpeed;

      Vector3D dir = point_mass.speed;
      dir.normalize();
      point_mass.speed = dir * CGL::clamp(point_mass.speed.norm(), point_mass.minSpeed, point_mass.maxSpeed);

      point_mass.cumulatedSpeed = 0;
      for (CollisionObject *collision_object : *collision_objects)
      {
        collision_object->collide(point_mass);
      }
      point_mass.position += point_mass.speed;
      // std::cout << isnan(point_mass.position.x) << endl;
      if (isnan(point_mass.position.x))
      {
        point_mass = PointMass(Vector3D(rand() % 100 / 100., rand() % 100 / 100., rand() % 100 / 100.), false);
      }
    }
  }

  for (PointMass &point_mass : point_masses)
  {
    if (is_stopped)
    {
      vector<Vector3f> line = stopLine[point_mass.branch];
      Vector3D a(line[0][0], line[0][1], line[0][2]);
      Vector3D b(line[1][0], line[1][1], line[1][2]);
      double dis = cylinder->computeDistance(point_mass.position, a, b);
      if (dis < 0.5)// && point_mass.timer > 0)//point_mass.position[1] > a[1] && point_mass.position[1] > b[1] && 
      {
        // point_mass.timer -= 1;
        if (point_mass.rand_stop_pos == NULL)
        {
          double x = (double)(rand() % 74) / 100. + .13; // cut first and last 13%
          point_mass.rand_stop_pos = a + (b - a) * x;
          point_mass.rand_stop_pos[1] += 0.02;
        }
        point_mass.speed = 0.00025 * (point_mass.rand_stop_pos - point_mass.position);
        point_mass.position += point_mass.speed;
      }
    }
    else
    {
      // if "S" pressed for second time, clear random stop position
      point_mass.rand_stop_pos = NULL;
      // point_mass.speed = generatePos();
    }
    // if (point_mass.timer <= 0 && (random() % 100 / 100. < 0.05)) {
    //   point_mass.timer = 100;
    // }
    // rr point_mass.rand_stop_pos = NULL;
  }
}

Vector3D Flock::accelerationAgainstWall(double distance, Vector3D direction)
{
  double encounterDis = 0.05;
  double wallWeight = 20;
  if (distance < encounterDis)
  {
    return direction * (wallWeight / abs(distance / encounterDis));
  }
  return Vector3D();
}

void Flock::build_spatial_map()
{
  for (const auto &entry : map)
  {
    delete (entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
  for (PointMass &point_mass : point_masses)
  {
    float hashed = hash_position(point_mass.position);
    if (!map[hashed])
    {
      map[hashed] = new vector<PointMass *>();
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
  if (map[hashed])
  {
    for (PointMass *point_mass : *map[hashed])
    {
      Vector3D dis = pm.position - point_mass->position;
      if (point_mass != &pm && dis.norm() <= 2 * thickness)
      {
        temp += (2 * thickness - dis.norm()) * dis.unit();
        i += 1;
      }
    }
  }
  if (i)
  {
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
  return x + y * 233. + z * 233. * 233.; // need mod?
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Flock::follow()
{
  for (PointMass &p : point_masses)
  {
    Vector3D dir = (cursor.position - p.position);
    p.cumulatedSpeed = dir;
  }
}

void Flock::reset()
{
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++)
  {
    pm->cumulatedSpeed = Vector3D();
    initializeSpeed(pm);
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
