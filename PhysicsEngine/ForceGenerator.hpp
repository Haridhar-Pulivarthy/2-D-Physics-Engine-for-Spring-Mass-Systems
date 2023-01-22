#pragma once

#include "SystemState.hpp"

// This structure applies forces to the objects in the scene.
// This is an abstract class from which other force generators (e.g. gravity) will be derived.

class ForceGenerator
{
public:
    ForceGenerator();
    virtual ~ForceGenerator();
    virtual void apply(SystemState* system) = 0;

    int m_index;
};