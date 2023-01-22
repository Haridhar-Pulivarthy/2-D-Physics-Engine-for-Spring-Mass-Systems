#include "RigidBody.hpp"

RigidBody::RigidBody()
{
    index = -1;
    reset();
}

RigidBody::~RigidBody() {}

double RigidBody::energy() const
{
    const double speed_2 = velX * velX + velY * velY;
    const double E_k = 0.5 * m * speed_2;
    const double E_r = 0.5 * I * angularVelocity * angularVelocity;

    return E_k + E_r;
}

void RigidBody::localToWorld(double x, double y, double* w_x, double* w_y)
{
    const double cosTheta = cos(orientation);
    const double sinTheta = sin(orientation);

    *w_x = cosTheta * x - sinTheta * y + posX;
    *w_y = sinTheta * x + cosTheta * y + posY;
}

void RigidBody::worldToLocal(double x, double y, double* l_x, double* l_y)
{
    const double cosTheta = cos(orientation);
    const double sinTheta = sin(orientation);

    *l_x = cosTheta * (x - posX) + sinTheta * (y - posY);
    *l_y = -sinTheta * (x - posX) + cosTheta * (y - posY);
}

void RigidBody::reset() {
    posX = posY = 0.0;
    velX = velY = 0.0;

    orientation = 0.0;
    angularVelocity = 0.0;

    m = 0.0;
    I = 0.0;
}
