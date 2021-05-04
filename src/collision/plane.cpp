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
    if (dot(pm.last_position - point1, normal) * dot(pm.position - point1, normal) <= 0) {
        Vector3D tangent_point1 = dot(point1 - pm.position, normal.unit()) * normal.unit() + pm.position;
        double t = (dot(normal, point1) - dot(normal, pm.position)) / dot(normal, normal);
        Vector3D tangent_point = pm.position + normal * t;
        Vector3D correction = (tangent_point + (SURFACE_OFFSET * normal)) - pm.last_position;
        pm.position = pm.last_position + correction * (1 - friction);
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
