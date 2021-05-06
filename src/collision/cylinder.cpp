#include "iostream"
#include <nanogui/nanogui.h>

#include "../flockMesh.h"
#include "../flockSimulator.h"
#include "cylinder.h"

using namespace std;
using namespace CGL;

#define SURFACE_OFFSET 0.0001

Vector3D Cylinder::getProjected(Vector3D A, Vector3D B, Vector3D C) {
  Vector3D d = (C - B) / (C - B).norm();
  Vector3D v = A - B;
  double t = dot(v, d);
  return B + t * d;
}

double Cylinder::computeDistance(Vector3D A, Vector3D B, Vector3D C) {
  return (getProjected(A, B, C) - A).norm();
}

Vector3f convert(Matrix3x3 m, Vector3f p1)
{
  Vector3D temp = m * Vector3D(p1[0], p1[1], p1[2]);
  return Vector3f(temp[0], temp[1], temp[2]);
}

void Cylinder::collide(PointMass &pm)
{
  Vector3D nextP = pm.position + pm.speed;
  Vector3D speed = pm.speed;
  double birdR = 0.03;
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
    Vector3f base = Vector3f(point.x, point.y, point.z);
    Vector3f p1 = convert(m2, convert(m1, Vector3f(0.0, l, 0.0))) + base;
    Vector3f p6 = convert(m2, convert(m1, Vector3f(0.0, -l, 0.0))) + base;
    if (p1[1] < p6[1]) {
      swap(p1, p6);
    }
    ax1 = min(p1[0], p6[0]) - r;
    ax2 = max(p1[0], p6[0]) + r;
    ay1 = min(p1[0], p6[0]) - r;
    ay2 = max(p1[0], p6[0]) + r;
    az1 = min(p1[0], p6[0]) - r;
    az2 = max(p1[0], p6[0]) + r;

    bx1 = nextP.x - birdR;
    bx2 = nextP.x + birdR;
    by1 = nextP.y - birdR;
    by2 = nextP.y + birdR;
    bz1 = nextP.z - birdR;
    bz2 = nextP.z + birdR;
    Vector3D a = getProjected(pm.position, Vector3D(p1[0], p1[1], p1[2]), Vector3D(p6[0], p6[1], p6[2]));
    Vector3D b = getProjected(nextP, Vector3D(p1[0], p1[1], p1[2]), Vector3D(p6[0], p6[1], p6[2]));

    if (ax2 > bx1 && ax1 < bx2 && ay2 > by1 && ay1 < by2 && az2 > bz1 && az1 < bz2)
    { // intersect
      // pm.speed *= -1.;
    } else {
      // pm.speed = speed;
    }
  }
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
    Vector3f top = Vector3f(0, 0, 0), bot = Vector3f(0, 0, 0);
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
      if (p2[1] > top[1]) {
        top = p2;
        bot = p5;
      }
      if (p3[1] > top[1]) {
        top = p3;
        bot = p4;
      }
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
    if (index > 1 && stopLine.size() < num_branch) { // not add the first cylinder which is the body of the tree
      vector<nanogui::Vector3f> temp2{top, bot};
      stopLine.push_back(temp2);
    }
    // cout << "point " << index << " top: x: " << top[0] << " y: " << top[1] << " z: " << top[2] << endl;
    // cout << "point " << index << " bot: x: " << bot[0] << " y: " << bot[1] << " z: " << bot[2] << endl;
  }
}
