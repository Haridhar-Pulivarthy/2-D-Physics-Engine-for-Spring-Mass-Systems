#pragma once

#include "ForceGenerator.hpp"
#include "RigidBody.hpp"

// This is a force generator for static forces.
// It is a subclass of the abstract class ForceGenerator.

class StaticForces : public ForceGenerator {
public:
    StaticForces();
    virtual ~StaticForces();

    virtual void apply(SystemState* state);

    void setForce(double forX, double forY);
    void setPosition(double posX, double posY);

    double m_forX;
    double m_forY;

    double m_posX;
    double m_posY;

    RigidBody* m_body;
};