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
  Cylinder(const Vector3D &point1, const Vector3D &normal, double radius, double halfLength, int slices, double friction)
      : point1(point1), normal(normal), radius(radius), halfLength(halfLength), slices(slices), friction(friction) {}

  void render(GLShader &shader);
  void collide(PointMass &pm);

  Vector3D point1;
  Vector3D normal;
  double radius;
  double halfLength;
  int slices;
  double friction;
};

#endif /* COLLISIONOBJECT_CYLINDER_H */
