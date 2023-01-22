#pragma once

#include <cmath>;

using std::cos;
using std::sin;

// Simulating a single rigid body.

struct RigidBody
{
public:
    RigidBody();
    ~RigidBody();

    void localToWorld(double x, double y, double* w_x, double* w_y);
    void worldToLocal(double x, double y, double* l_x, double* l_y);

    double posX;
    double posY;

    double velX;
    double velY;

    double orientation;
    double angularVelocity;

    double m;
    double I;

    int index;

    void reset();
    double energy() const;
};