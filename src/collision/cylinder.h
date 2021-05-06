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
  Cylinder();
  Cylinder(const vector<Vector3D> &points, const vector<vector<double> > &rotates, const vector<double> &radius, const vector<double> &halfLength, int slices, double friction, int branchNum, int poleNum)
      : points(points), rotates(rotates), radius(radius), halfLength(halfLength), slices(slices), friction(friction), branchNum(branchNum), poleNum(poleNum){}

  void render(GLShader &shader);
  void collide(PointMass &pm);

  vector<Vector3D> points;
  vector<vector<double> > rotates;
  vector<double> radius;
  vector<double> halfLength;
  int slices;
  double friction;
  vector<vector<Vector3f> > stopLine;

  Vector3D getProjected(Vector3D A, Vector3D B, Vector3D C);
  double computeDistance(Vector3D A, Vector3D B, Vector3D C);
  int branchNum;
  int poleNum;
};

#endif /* COLLISIONOBJECT_CYLINDER_H */
