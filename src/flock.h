#ifndef CLOTH_H
#define CLOTH_H

#include <unordered_set>
#include <unordered_map>
#include <vector>

#include "bird.h"
#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "flockMesh.h"
#include "collision/collisionObject.h"
#include "spring.h"

using namespace CGL;
using namespace std;

enum e_orientation { HORIZONTAL = 0, VERTICAL = 1 };

struct FlockParameters {
   FlockParameters() {}
   FlockParameters(double coherence,
                  double alignment,
                   double separation)
      : coherence(coherence), alignment(alignment), separation(separation) {}
  ~FlockParameters() {}

  // Global simulation parameters

  /*bool enable_structural_constraints;
  bool enable_shearing_constraints;
  bool enable_bending_constraints;*/

  // flock parameters
  double coherence;
  double alignment;
  double separation;
  int num_birds = 50;
};

struct Flock {
  Flock() {}
  Flock(double width, double height, int num_width_points,
        int num_height_points, float thickness);
  ~Flock();

  void buildGrid();

  void simulate(double frames_per_sec, double simulation_steps, FlockParameters *fp,
                vector<Vector3D> external_accelerations,
                vector<CollisionObject *> *collision_objects, Vector3D windDir);
  vector<PointMass> getNeighbours(PointMass pm, double range);
  void reset();
  void buildFlockMesh();

  void build_spatial_map();
  void self_collide(PointMass &pm, double simulation_steps);
  float hash_position(Vector3D pos);


  Vector3D accelerationAgainstWall(double distance, Vector3D direction);
  void follow();
  Vector3D generatePos();

  // flock properties
  double width;
  double height;
  int num_width_points;
  int num_height_points;
  double thickness;
  e_orientation orientation;
  bool following = false;

  // flock components
  vector<PointMass> point_masses;
  vector<Bird> birds;
  vector<vector<int>> pinned;
  vector<Spring> springs;
  FlockMesh *flockMesh;
  PointMass cursor = PointMass(Vector3D(0.5, 0.5, 0.5), false);

  // Spatial hashing
  unordered_map<float, vector<PointMass *> *> map;
  double x = 1;
  double y = 1;
  double z = 1;

  int num_birds = 50;//20 - 1000

};

#endif /* CLOTH_H */
