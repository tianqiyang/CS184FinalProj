#ifndef COLLISIONOBJECT_PLANE_H
#define COLLISIONOBJECT_PLANE_H

#include <nanogui/nanogui.h>

#include "../flockMesh.h"
#include "collisionObject.h"

using namespace nanogui;
using namespace CGL;
using namespace std;

struct Plane : public CollisionObject {
public:
  Plane(const Vector3D &point1, const Vector3D &point2, const Vector3D &point3,
        const Vector3D &point4, const Vector3D &normal, double friction)
      : point1(point1), point2(point2), point3(point3), point4(point4),
  normal(normal.unit()), friction(friction) {}

  void render(GLShader &shader);
  void collide(PointMass &pm);

  Vector3D point1;
  Vector3D point2;
  Vector3D point3;
  Vector3D point4;
  Vector3D normal;

  double friction;
};

#endif /* COLLISIONOBJECT_PLANE_H */
