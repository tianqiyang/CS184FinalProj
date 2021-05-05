#ifndef COLLISIONOBJECT_CYLINDER_H
#define COLLISIONOBJECT_CYLINDER_H

#include <nanogui/nanogui.h>

#include "../flockMesh.h"
#include "collisionObject.h"

using namespace nanogui;
using namespace CGL;
using namespace std;

struct Cylinder : public CollisionObject {
public:
  Cylinder(const vector<Vector3D> &points, const vector<vector<double> > &rotates, const vector<double> &radius, const vector<double> &halfLength, int slices, double friction)
      : points(points), rotates(rotates), radius(radius), halfLength(halfLength), slices(slices), friction(friction) {}

  void render(GLShader &shader);
  void collide(PointMass &pm);

  vector<Vector3D> points;
  vector<vector<double> > rotates;
  vector<double> radius;
  vector<double> halfLength;
  int slices;
  double friction;
};

#endif /* COLLISIONOBJECT_CYLINDER_H */
