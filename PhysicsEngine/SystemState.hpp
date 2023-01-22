#pragma once

#include <assert.h>
#include <cstring>
#include <cmath>

using std::memcpy;
using std::sin;
using std::cos;

// This structure tracks various object properties, e.g. position, velocity, and orientation.

class SystemState
{
public:
    SystemState();
    ~SystemState();

    void copy(const SystemState* state);
    void resize(int bodyCount, int constraintCount);
    void destroy();

    void localToWorld(double x, double y, double* xt, double* yt, int body);
    void velocityAtPoint(double x, double y, double* velX, double* velY, int body);
    void applyForce(double xl, double yl, double fx, double fy, int body);

    int* indexMap;

    double* angularAcceleration;
    double* angularVelocity;
    double* orientation;

    double* accX;
    double* accY;
    double* velX;
    double* velY;
    double* posX;
    double* posY;

    double* f_x;
    double* f_y;
    double* t;

    double* r_x;
    double* r_y;
    double* r_t;

    double* m;

    int n;
    int n_c;
    double dt;
};

void freeArray(double*& data);
void freeArray(int*& data);