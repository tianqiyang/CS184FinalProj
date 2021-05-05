#ifndef POINTMASS_H
#define POINTMASS_H

#include "CGL/CGL.h"
#include "CGL/misc.h"
#include "CGL/vector3D.h"

using namespace CGL;

// Forward declarations
class Halfedge;

struct PointMass {
    PointMass(Vector3D position, bool pinned)
        : pinned(pinned), start_position(position), position(position),
        last_position(position), speed(Vector3D(0.0001, 0, 0)) {}

    Vector3D normal();
    Vector3D velocity(double delta_t) {
        return (position - last_position) / delta_t;
    }

    // static values
    bool pinned;
    Vector3D start_position;

    // dynamic values
    Vector3D position;
    Vector3D last_position;
    Vector3D forces;

    // mesh reference
    Halfedge* halfedge;

    // speed
    Vector3D speed = Vector3D();
    Vector3D cumulatedSpeed;

    double minSpeed = .0002;
    double maxSpeed = .0004;
    double maxAcc = 1.;
    double minAcc = 0.;
  
    Vector3D rand_stop_pos;
};

#endif /* POINTMASS_H */
