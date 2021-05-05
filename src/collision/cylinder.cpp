#include "iostream"
#include <nanogui/nanogui.h>

#include "../flockMesh.h"
#include "../flockSimulator.h"
#include "cylinder.h"

using namespace std;
using namespace CGL;

#define SURFACE_OFFSET 0.0001

Vector3D getProjected(Vector3D A, Vector3D B, Vector3D C) {
  Vector3D d = (C - B) / (C - B).norm();
  Vector3D v = A - B;
  double t = dot(v, d);
  return B + t * d;
}

double computeDistance(Vector3D A, Vector3D B, Vector3D C) {
  return (getProjected(A, B, C) - A).norm();
}

Vector3f convert(Matrix3x3 m, Vector3f p1)
{
  Vector3D temp = m * Vector3D(p1[0], p1[1], p1[2]);
  return Vector3f(temp[0], temp[1], temp[2]);
}

void Cylinder::collide(PointMass &pm)
{
  /*
  Vector3D nextP = pm.position + pm.speed;
  Vector3D speed = pm.speed;
  // normal == 0 pole range: x: 1 +- r, y: 0.6 +- r, z: -.5 to 1
  //       other pole range: x: 1 +- r, y: -1 to 1, z: 1 +- r
  double ax1, ax2, bx1, bx2, ay1, ay2, by1, by2, az1, az2, bz1, bz2;
  for (int index = 0; index < points.size(); index++)
  {
    Vector3D point = points[index];
    vector<double> rotate = rotates[index];
    double r = radius[index];
    double l = halfLength[index];
    double turn = PI * rotate[0] / 180.;
    double dataArray1[9] = {cos(turn), sin(turn) * -1., 0., sin(turn), cos(turn), 0., 0., 0., 1.};
    turn = PI * rotate[1] / 180.;
    double dataArray2[9] = {cos(turn), 0., sin(turn), 0., 1., 0., sin(turn) * -1., 0., cos(turn)};
    double *data1 = dataArray1;
    double *data2 = dataArray2;
    CGL::Matrix3x3 m1 = CGL::Matrix3x3(data1);
    CGL::Matrix3x3 m2 = CGL::Matrix3x3(data2);
    Vector3f p1 = convert(m2, convert(m1, Vector3f(0.0, l, point.z)));
    Vector3f p6 = convert(m2, convert(m1, Vector3f(0.0, -l-point.y, point.z)));
    ax1 = min(p1[0], p6[0]) - r;
    ax2 = max(p1[0], p6[0]) + r;
    ay1 = min(p1[0], p6[0]);
    ay2 = max(p1[0], p6[0]);
    az1 = min(p1[0], p6[0]) - r;
    az2 = max(p1[0], p6[0]) + r;
    // nextP: range +- 0.02
    bx1 = nextP.x - .02;
    bx2 = nextP.x + .02;
    by1 = nextP.y - .02;
    by2 = nextP.y + .02;
    bz1 = nextP.z - .02;
    bz2 = nextP.z + .02;
    Vector3D a = getProjected(pm.position, Vector3D(p1[0], p1[1], p1[2]), Vector3D(p6[0], p6[1], p6[2]));
    Vector3D b = getProjected(nextP, Vector3D(p1[0], p1[1], p1[2]), Vector3D(p6[0], p6[1], p6[2]));


    if (ax2 > bx1 && ax1 < bx2 && ay2 > by1 && ay1 < by2 && az2 > bz1 || az1 < bz2)
    {
      pm.speed *= -1.;
    // } else if (computeDistance()) {
      
    } else {
      pm.speed = speed;
    }
  }
  */
}


void Cylinder::render(GLShader &shader)
{
  nanogui::Color color(0.7f, 0.7f, 0.0f, 1.0f);
  for (int index = 0; index < points.size(); index++)
  {
    Vector3D point = points[index];
    vector<double> rotate = rotates[index];
    double r = radius[index];
    double l = halfLength[index];
    for (int i = 0; i < slices; i++)
    {
      MatrixXf positions(3, 6);
      MatrixXf normals(3, 6);
      float theta = 2.0 * PI * ((float)i) / slices;
      float nextTheta = 2.0 * PI * ((float)i + 1) / slices;
      double turn = PI * rotate[0] / 180.;
      double dataArray1[9] = {cos(turn), sin(turn) * -1., 0., sin(turn), cos(turn), 0., 0., 0., 1.};
      turn = PI * rotate[1] / 180.;
      double dataArray2[9] = {cos(turn), 0., sin(turn), 0., 1., 0., sin(turn) * -1., 0., cos(turn)};
      double *data1 = dataArray1;
      double *data2 = dataArray2;
      CGL::Matrix3x3 m1 = CGL::Matrix3x3(data1);
      CGL::Matrix3x3 m2 = CGL::Matrix3x3(data2);
      Vector3f base = Vector3f(point.x, point.y, point.z);
      Vector3f p1 = convert(m2, convert(m1, Vector3f(0.0, l, 0.0))) + base;
      Vector3f p2 = convert(m2, convert(m1, Vector3f(0.0 + r * cos(theta), l, r * sin(theta)))) + base;
      Vector3f p3 = convert(m2, convert(m1, Vector3f(0.0 + r * cos(nextTheta), l, r * sin(nextTheta)))) + base;
      Vector3f p4 = convert(m2, convert(m1, Vector3f(0.0 + r * cos(nextTheta), -l, r * sin(nextTheta)))) + base;
      Vector3f p5 = convert(m2, convert(m1, Vector3f(0.0 + r * cos(theta), -l, r * sin(theta)))) + base;
      Vector3f p6 = convert(m2, convert(m1, Vector3f(0.0, -l, 0.0))) + base;
      Vector3f n1 = convert(m2, convert(m1, Vector3f(cos(theta), 0.0, sin(theta))));
      Vector3f n2 = convert(m2, convert(m1, Vector3f(0.0, 1.0, 0.0)));
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
