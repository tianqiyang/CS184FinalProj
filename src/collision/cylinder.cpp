#include "iostream"
#include <nanogui/nanogui.h>

#include "../flockMesh.h"
#include "../flockSimulator.h"
#include "cylinder.h"

using namespace std;
using namespace CGL;

#define SURFACE_OFFSET 0.0001

void Cylinder::collide(PointMass &pm) {
  Vector3D nextP = pm.position + pm.speed;
  Vector3D speed = pm.speed;
  // normal == 0 pole range: x: 1 +- radius, y: 0.6 +- radius, z: -.5 to 1
  //       other pole range: x: 1 +- radius, y: -1 to 1, z: 1 +- radius
  double ax1, ax2, bx1, bx2, ay1, ay2, by1, by2, az1, az2, bz1, bz2;
  if (normal == 0) {
    ax1 = 1 - radius;
    ax2 = 1 + radius;
    ay1 = .6 - radius;
    ay2 = .6 + radius;
    az1 = -.5;
    az2 = 1;
  } else {
    ax1 = 1 - radius;
    ax2 = 1 + radius;
    ay1 = -1;
    ay2 = 1;
    az1 = 1 - radius;
    az2 = 1 + radius;
  }
  // nextP: range +- 0.02
  bx1 = nextP.x - .02;
  bx2 = nextP.x + .02;
  by1 = nextP.y - .02;
  by2 = nextP.y + .02;
  bz1 = nextP.z - .02;
  bz2 = nextP.z + .02;

  // wrong boundary
  // bx1 = min(pm.position.x, nextP.x) - birdRadius;
  // bx2 = max(pm.position.x, nextP.x) + birdRadius;
  // by1 = min(pm.position.y, nextP.y) - birdRadius;
  // by2 = max(pm.position.y, nextP.y) + birdRadius;
  // bz1 = min(pm.position.z, nextP.z) - birdRadius;
  // bz2 = max(pm.position.z, nextP.z) + birdRadius;
  // if (ax2 > bx1 || ax1 < bx2) { // overlap in x plane
  //   speed.x = 0;
  // }
  // if (ay2 > by1 || ay1 < by2) {
  //   speed.y = 0;
  // }
  // if (az2 > bz1 || az1 < bz2) {
  //   speed.z = 0;
  // }
  if (ax2 > bx1 && ax1 < bx2 && ay2 > by1 && ay1 < by2 && az2 > bz1 || az1 < bz2) {
    pm.speed *= -1.;
  }
  // double length = speed.norm();
  // if (length == 0) { // they are overlapped in 3d
    // TODO: find the perpendicular to c
  // } else {
  // }
    pm.speed = speed;
}

void Cylinder::render(GLShader &shader) {
  nanogui::Color color(0.7f, 0.7f, 0.0f, 1.0f);
  int count = 6;
  for(int i=0; i < slices; i++) { 
    MatrixXf positions(3, count);
    MatrixXf normals(3, count);
    float theta = 2.0 * PI * ((float)i) / slices;
    float nextTheta = 2.0 * PI * ((float)i+1) / slices;
    Vector3f p1 = Vector3f(point1.x, halfLength + point1.y, point1.z); // 1, 1, 1
    Vector3f p2 = Vector3f(point1.x+radius*cos(theta), halfLength + point1.y, point1.z + radius*sin(theta));
    Vector3f p3 = Vector3f (point1.x+radius*cos(nextTheta), halfLength + point1.y, point1.z + radius*sin(nextTheta));
    Vector3f p4 = Vector3f(point1.x+radius*cos(nextTheta), -halfLength , point1.z + radius*sin(nextTheta));
    Vector3f p5= Vector3f(point1.x+radius*cos(theta), -halfLength, point1.z + radius*sin(theta));
    Vector3f p6= Vector3f(point1.x, -halfLength, point1.z); // 1, -1, 1

    Vector3f n1 = Vector3f(cos(theta), 0.0, sin(theta));
    Vector3f n2 = Vector3f(1.0, 0.0, 0.0);
    if (normal == 0) {
      p1 = Vector3f(p1[0], -p1[2], p1[1]); // 1, 0.6, 1
      p2 = Vector3f(p2[0], -p2[2], p2[1]);
      p3 = Vector3f(p3[0], -p3[2], p3[1]);
      p4 = Vector3f(p4[0], -p4[2], p4[1]);
      p5 = Vector3f(p5[0], -p5[2], p5[1]);
      p6 = Vector3f(p6[0], -p6[2], p6[1]); //1, 0.6, -0.5
      n1 = Vector3f(cos(theta), 1.0, sin(theta)); 
      n2 = Vector3f(1.0, 0.0, 0.0);
    }
    // cout << p1[0] << " y: " << p1[1] << " z: " << p1[2] << endl;
    // cout << p6[0] << " y: " << p6[1] << " z: " << p6[2] << endl;
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
  }
  
}
