#include "iostream"
#include <nanogui/nanogui.h>

#include "../flockMesh.h"
#include "../flockSimulator.h"
#include "cylinder.h"

using namespace std;
using namespace CGL;

#define SURFACE_OFFSET 0.0001

void Cylinder::collide(PointMass &pm)
{
  /*
  Vector3D nextP = pm.position + pm.speed;
  Vector3D speed = pm.speed;
  // normal == 0 pole range: x: 1 +- r, y: 0.6 +- r, z: -.5 to 1
  //       other pole range: x: 1 +- r, y: -1 to 1, z: 1 +- r
  double ax1, ax2, bx1, bx2, ay1, ay2, by1, by2, az1, az2, bz1, bz2;
  if (normal == 0)
  {
    ax1 = 1 - r;
    ax2 = 1 + r;
    ay1 = .6 - r;
    ay2 = .6 + r;
    az1 = -.5;
    az2 = 1;
  }
  else
  {
    ax1 = 1 - r;
    ax2 = 1 + r;
    ay1 = -1;
    ay2 = 1;
    az1 = 1 - r;
    az2 = 1 + r;
  }
  // nextP: range +- 0.02
  bx1 = nextP.x - .02;
  bx2 = nextP.x + .02;
  by1 = nextP.y - .02;
  by2 = nextP.y + .02;
  bz1 = nextP.z - .02;
  bz2 = nextP.z + .02;

  // wrong boundary
  // bx1 = min(pm.position.x, nextP.x) - birdr;
  // bx2 = max(pm.position.x, nextP.x) + birdr;
  // by1 = min(pm.position.y, nextP.y) - birdr;
  // by2 = max(pm.position.y, nextP.y) + birdr;
  // bz1 = min(pm.position.z, nextP.z) - birdr;
  // bz2 = max(pm.position.z, nextP.z) + birdr;
  // if (ax2 > bx1 || ax1 < bx2) { // overlap in x plane
  //   speed.x = 0;
  // }
  // if (ay2 > by1 || ay1 < by2) {
  //   speed.y = 0;
  // }
  // if (az2 > bz1 || az1 < bz2) {
  //   speed.z = 0;
  // }
  if (ax2 > bx1 && ax1 < bx2 && ay2 > by1 && ay1 < by2 && az2 > bz1 || az1 < bz2)
  {
    pm.speed *= -1.;
  }
  // double length = speed.norm();
  // if (length == 0) { // they are overlapped in 3d
  // TODO: find the perpendicular to c
  // } else {
  // }
  pm.speed = speed;
  */
}

Vector3f convert(Matrix3x3 m, Vector3f p1)
{
  Vector3D temp = m * Vector3D(p1[0], p1[1], p1[2]);
  return Vector3f(temp[0], temp[1], temp[2]);
}

void Cylinder::render(GLShader &shader)
{
  nanogui::Color color(0.7f, 0.7f, 0.0f, 1.0f);
  for (int index = 0; index < points.size(); index++)
  {
    Vector3D point = points[index];
    double rotate = rotates[index];
    double r = radius[index];
    double l = halfLength[index];
    for (int i = 0; i < slices; i++)
    {
      MatrixXf positions(3, 6);
      MatrixXf normals(3, 6);
      float theta = 2.0 * PI * ((float)i) / slices;
      float nextTheta = 2.0 * PI * ((float)i + 1) / slices;
      double turn = PI * rotate / 180.;
      double dataArray[9] = {cos(turn), sin(turn) * -1., 0., sin(turn), cos(turn), 0., 0., 0., 1.};
      double *data = dataArray;
      CGL::Matrix3x3 m = CGL::Matrix3x3(data);
      Vector3f p1 = convert(m, Vector3f(point.x, l, point.z));
      Vector3f p2 = convert(m, Vector3f(point.x + r * cos(theta), l, point.z + r * sin(theta)));
      Vector3f p3 = convert(m, Vector3f(point.x + r * cos(nextTheta), l, point.z + r * sin(nextTheta)));
      Vector3f p4 = convert(m, Vector3f(point.x + r * cos(nextTheta), -l, point.z + r * sin(nextTheta)));
      Vector3f p5 = convert(m, Vector3f(point.x + r * cos(theta), -l, point.z + r * sin(theta)));
      Vector3f p6 = convert(m, Vector3f(point.x, -l, point.z));
      Vector3f n1 = convert(m, Vector3f(cos(theta), 0.0, sin(theta)));
      Vector3f n2 = convert(m, Vector3f(0.0, 1.0, 0.0));
      p1[1] += point.y;
      p2[1] += point.y;
      p3[1] += point.y;
      p4[1] += point.y;
      p5[1] += point.y;
      p6[1] += point.y;
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

      if (shader.uniform("u_color", false) != -1)
      {
        shader.setUniform("u_color", color);
      }
      shader.uploadAttrib("in_position", positions);
      if (shader.attrib("in_normal", false) != -1)
      {
        shader.uploadAttrib("in_normal", normals);
      }
      shader.drawArray(GL_TRIANGLE_STRIP, 0, 6);
    }
  }
}
