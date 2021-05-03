#include "iostream"
#include <nanogui/nanogui.h>

#include "../flockMesh.h"
#include "../flockSimulator.h"
#include "cylinder.h"

using namespace std;
using namespace CGL;

#define SURFACE_OFFSET 0.0001

void Cylinder::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with planes.
    if (dot(pm.last_position - point1, normal) * dot(pm.position - point1, normal) <= 0) {
        Vector3D tangent_point1 = dot(point1 - pm.position, normal.unit()) * normal.unit() + pm.position;
        double t = (dot(normal, point1) - dot(normal, pm.position)) / dot(normal, normal);
        Vector3D tangent_point = pm.position + normal * t;
        Vector3D correction = (tangent_point + (SURFACE_OFFSET * normal)) - pm.last_position;
        pm.position = pm.last_position + correction * (1 - friction);
    }
}

void Cylinder::render(GLShader &shader) {
  nanogui::Color color(0.7f, 0.7f, 0.0f, 1.0f);
  double halfLength = .5;
  double radius = .5;
  int count = 6;// * slices;
  MatrixXf positions(3, count);
  MatrixXf normals(3, count);


  // float theta = ((float)i)*2.0*PI;
  // float nextTheta = ((float)i+1)*2.0*PI;

  Vector3f p1 = Vector3f(-1, halfLength, -1);
  Vector3f p2 = Vector3f(-1, halfLength, 2);
  Vector3f p3 = Vector3f (2, halfLength, -1);
  Vector3f p4 = Vector3f(2, -halfLength, -1);
  Vector3f p5= Vector3f(-1, -halfLength, 2);
  Vector3f p6= Vector3f(-1, -halfLength, -1);

  Vector3f n1 = Vector3f(0.0, -1.0, 0.0);
  Vector3f n2 = Vector3f(1.0, 0.0, 0.0);

  positions.col(0) << p1;
  positions.col(1) << p2;
  positions.col(2) << p3;
  positions.col(3) << p5;
  positions.col(4) << p4;
  positions.col(5) << p6;

  normals.col(0) << n1;
  normals.col(1) << n1;
  normals.col(2) << n1;
  normals.col(3) << n2;
  normals.col(4) << n2;
  normals.col(5) << n1;

  
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

  shader.drawArray(GL_TRIANGLE_STRIP, 0, count);
  // Vector3f sPoint1(point1.x, point1.y, point1.z);
  // Vector3f sNormal(normal.x, normal.y, normal.z);

  // int num_tri = 6;//6 * slices;
  // MatrixXf positions(3, num_tri);
  // MatrixXf normals(3, 4);
  
  // for(int i=0; i < 1; i++) { 
  //   float theta = ((float)i)*2.0*PI;
  //   float nextTheta = ((float)i+1)*2.0*PI;

  //   /*vertex at middle of end */ 
  //   Vector3D p1 = Vector3D(0.0, halfLength, 0.0);
  //   /*vertices at edges of circle*/ 
  //   Vector3D p2 = Vector3D(radius*cos(theta), halfLength, radius*sin(theta));
  //   Vector3D p3 = Vector3D (radius*cos(nextTheta), halfLength, radius*sin(nextTheta));
  //   // /* the same vertices at the bottom of the cylinder*/
  //   Vector3D p4 = Vector3D (radius*cos(nextTheta), -halfLength, radius*sin(nextTheta));
  //   Vector3D p5 = Vector3D(radius*cos(theta), -halfLength, radius*sin(theta));
  //   Vector3D p6 = Vector3D(0.0, -halfLength, 0.0);

  //   positions.col(i * 3 + 0) << p1.x, p1.y, p1.z, 1.0;
  //   positions.col(i * 3 + 1) << p2.x, p2.y, p2.z, 1.0;
  //   positions.col(i * 3 + 2) << p3.x, p3.y, p3.z, 1.0;
  //   positions.col(i * 3 + 3) << p4.x, p4.y, p4.z, 1.0;
  //   positions.col(i * 3 + 4) << p5.x, p5.y, p5.z, 1.0;
  //   positions.col(i * 3 + 5) << p6.x, p6.y, p6.z, 1.0;

  //   Vector3D n1 = Vector3D(cos(theta), 0.0, sin(theta));
  //   Vector3D n2 = Vector3D(0.0, -1.0, 0.0);

  //   normals.col(i * 3 + 0) << n2.x, n2.y, n2.z, 0.0;
  //   normals.col(i * 3 + 1) << n1.x, n1.y, n1.z, 0.0;
  //   normals.col(i * 3 + 2) << n1.x, n1.y, n1.z, 0.0;
  //   normals.col(i * 3 + 3) << n2.x, n2.y, n2.z, 0.0;
  // }
  // //   Matrix4f model;
  // // model.setIdentity();
  // // shader.setUniform("u_model", model);

  // shader.setUniform("u_color", color, false);
  // shader.uploadAttrib("in_position", positions, false);
  // shader.uploadAttrib("in_normal", normals, false);
  // shader.drawArray(GL_TRIANGLE_STRIP, 1, 3);
}
