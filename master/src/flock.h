#ifndef FLOCK_H
#define FLOCK_H

#include <unordered_set>
#include <unordered_map>
#include <vector>

#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "clothMesh.h"
#include "collision/collisionObject.h"
#include "spring.h"
#include "bird.h"

using namespace CGL;
using namespace std;

struct FlockParameters {
  FlockParameters() {}
  FlockParameters(bool enable_structural_constraints,
                  bool enable_shearing_constraints,
                  bool enable_bending_constraints, double damping,
                  double density, double ks)
      : enable_structural_constraints(enable_structural_constraints),
        enable_shearing_constraints(enable_shearing_constraints),
        enable_bending_constraints(enable_bending_constraints),
        damping(damping), density(density), ks(ks) {}
  ~FlockParameters() {}

  // Global simulation parameters

  bool enable_structural_constraints;
  bool enable_shearing_constraints;
  bool enable_bending_constraints;

  double damping;

  // Mass-spring parameters
  double density;
  double ks;
};

struct Flock {
  Flock() {}
  Flock(double width, double height, int num_width_points,
        int num_height_points, float thickness);
  ~Flock();

  void buildGrid();

  void simulate(double frames_per_sec, double simulation_steps, FlockParameters *cp,
                vector<Vector3D> external_accelerations,
                vector<CollisionObject *> *collision_objects, Vector3D windDir);
  vector<PointMass> getNeighbours(PointMass pm, double range);
  void reset();
  void buildClothMesh();

  void build_spatial_map();
  void self_collide(PointMass &pm, double simulation_steps);
  float hash_position(Vector3D pos);

  // Cloth properties
  double width;
  double height;
  int num_width_points;
  int num_height_points;
  double thickness;

  // Cloth components
  vector<PointMass> point_masses;
  vector<Bird> birds;
  vector<vector<int>> pinned;
  vector<Spring> springs;
  ClothMesh *clothMesh;

  // Spatial hashing
  unordered_map<float, vector<PointMass *> *> map;
  double x = .5;
  double y = .5;
  double z = .5;

  int num_birds = 100; //20 - 1000

  double COHESION_RANGE = 0.067;
  double SEPARATION_RANGE = 0.05;
  double ALIGNMENT_RANGE = 0.05;
};

#endif /* CLOTH_H */
