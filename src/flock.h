#ifndef CLOTH_H
#define CLOTH_H

#include <unordered_set>
#include <unordered_map>
#include <vector>

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
};

struct Flock {
  Flock() {}
  Flock(double width, double height, int num_width_points,
        int num_height_points, float thickness);
  ~Flock();

  void buildGrid();

  void simulate(double frames_per_sec, double simulation_steps, FlockParameters *fp,
                vector<Vector3D> external_accelerations,
                vector<CollisionObject *> *collision_objects);

  void reset();
  void buildFlockMesh();

  void build_spatial_map();
  void self_collide(PointMass &pm, double simulation_steps);
  float hash_position(Vector3D pos);

  // Cloth properties
  double width;
  double height;
  int num_width_points;
  int num_height_points;
  double thickness;
  e_orientation orientation;

  // Cloth components
  vector<PointMass> point_masses;
  vector<vector<int>> pinned;
  vector<Spring> springs;
  FlockMesh *flockMesh;

  // Spatial hashing
  unordered_map<float, vector<PointMass *> *> map;
};

#endif /* CLOTH_H */
