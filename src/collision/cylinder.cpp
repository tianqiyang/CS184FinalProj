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
  Vector3D normalv (1,1,1);
    if (dot(pm.last_position - point1, normal) * dot(pm.position - point1, normal) <= 0) {
        Vector3D tangent_point1 = dot(point1 - pm.position, normalv.unit()) * normalv.unit() + pm.position;
        double t = (dot(normal, point1) - dot(normal, pm.position)) / dot(normalv, normalv);
        Vector3D tangent_point = pm.position + normal * t;
        Vector3D correction = (tangent_point + (SURFACE_OFFSET * normal)) - pm.last_position;
        pm.position = pm.last_position + correction * (1 - friction);
    }
}

void Cylinder::render(GLShader &shader) {
  nanogui::Color color(0.7f, 0.7f, 0.0f, 1.0f);
  int count = 6;
  for(int i=0; i < slices; i++) { 
    MatrixXf positions(3, count);
    MatrixXf normals(3, count);
    float theta = 2.0 * PI * ((float)i) / slices;
    float nextTheta = 2.0 * PI * ((float)i+1) / slices;
    Vector3f p1 = Vector3f(point1.x, halfLength + point1.y, point1.z);
    Vector3f p2 = Vector3f(point1.x+radius*cos(theta), halfLength + point1.y, point1.z + radius*sin(theta));
    Vector3f p3 = Vector3f (point1.x+radius*cos(nextTheta), halfLength + point1.y, point1.z + radius*sin(nextTheta));
    Vector3f p4 = Vector3f(point1.x+radius*cos(nextTheta), -halfLength , point1.z + radius*sin(nextTheta));
    Vector3f p5= Vector3f(point1.x+radius*cos(theta), -halfLength, point1.z + radius*sin(theta));
    Vector3f p6= Vector3f(point1.x, -halfLength, point1.z);

    Vector3f n1 = Vector3f(cos(theta), 0.0, sin(theta));
    Vector3f n2 = Vector3f(1.0, 0.0, 0.0);
    if (normal == 0) {
      p1 = Vector3f(p1[0], -p1[2], p1[1]);
      p2 = Vector3f(p2[0], -p2[2], p2[1]);
      p3 = Vector3f(p3[0], -p3[2], p3[1]);
      p4 = Vector3f(p4[0], -p4[2], p4[1]);
      p5 = Vector3f(p5[0], -p5[2], p5[1]);
      p6 = Vector3f(p6[0], -p6[2], p6[1]);
      n1 = Vector3f(cos(theta), 1.0, sin(theta));
      n2 = Vector3f(1.0, 0.0, 0.0);
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
