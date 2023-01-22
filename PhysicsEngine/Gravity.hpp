#pragma once

#include "ForceGenerator.hpp"
#include "RigidBody.hpp"

// This is a force generator for gravity.
// It is a subclass of the abstract class ForceGenerator.

class Gravity : public ForceGenerator {
public:
    Gravity();
    virtual ~Gravity();
    virtual void apply(SystemState* state);

    double mG;
};