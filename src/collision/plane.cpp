#include "iostream"
#include <nanogui/nanogui.h>

#include "../flockMesh.h"
#include "../flockSimulator.h"
#include "plane.h"

using namespace std;
using namespace CGL;

#define SURFACE_OFFSET 0.0001

void Plane::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with planes.
    Vector3D newPosition = pm.position + pm.speed;
    if (newPosition.z < .5 ) {
      pm.speed.z = .00000001;
    }
}

void Plane::render(GLShader &shader) {
  nanogui::Color color(0.7f, 0.7f, 0.7f, 1.0f);

  Vector3f sPoint1(point1.x, point1.y, point1.z);
  Vector3f sPoint2(point2.x, point2.y, point2.z);
  Vector3f sPoint3(point3.x, point3.y, point3.z);
  Vector3f sPoint4(point4.x, point4.y, point4.z);
  Vector3f sNormal(normal.x, normal.y, normal.z);
  Vector3f sParallel(normal.y - normal.z, normal.z - normal.x,
                     normal.x - normal.y);
  sParallel.normalize();
  Vector3f sCross = sNormal.cross(sParallel);

  MatrixXf positions(3, 4);
  MatrixXf normals(3, 4);

//  positions.col(0) << sPoint + 2 * (sCross + sParallel);
//  positions.col(1) << sPoint + 2 * (sCross - sParallel);
//  positions.col(2) << sPoint + 2 * (-sCross + sParallel);
//  positions.col(3) << sPoint + 2 * (-sCross - sParallel);
  
  positions.col(0) << sPoint1;
  positions.col(1) << sPoint2;
  positions.col(2) << sPoint3;
  positions.col(3) << sPoint4;

  normals.col(0) << sNormal;
  normals.col(1) << sNormal;
  normals.col(2) << sNormal;
  normals.col(3) << sNormal;
  
  Matrix4f model;
  model.setIdentity();
  shader.setUniform("u_model", model);

  if (shader.uniform("u_color", false) != -1) {
    shader.setUniform("u_color", color);
  }
  shader.uploadAttrib("in_position", positions);
  if (shader.attrib("in_normal", false) != -1) {
    shader.uploadAttrib("in_normal", normals);
  }

  shader.drawArray(GL_TRIANGLE_STRIP, 0, 4);
}
