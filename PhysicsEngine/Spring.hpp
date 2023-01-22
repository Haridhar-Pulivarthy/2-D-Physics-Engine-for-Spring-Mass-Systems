#pragma once

#include "ForceGenerator.hpp"
#include "RigidBody.hpp"

// This is a force generator for a spring object.
// It is a subclass of the abstract class ForceGenerator.

class Spring : public ForceGenerator {
public:
    Spring();
    virtual ~Spring();

    virtual void apply(SystemState* state);

    void getEnds(double* x1, double* y1, double* x2, double* y2);
    double energy() const;

    double m_restLength;
    double m_ks;
    double m_kd;

    double m_p1X;
    double m_p1Y;

    double m_p2X;
    double m_p2Y;

    RigidBody* m_body1;
    RigidBody* m_body2;
};