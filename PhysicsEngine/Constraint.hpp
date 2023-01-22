#pragma once

#include "SystemState.hpp"
#include "RigidBody.hpp"
#include "Matrix.hpp"

#include <assert.h>
#include <string.h>
#include <cfloat>

// Contains the calculate function, which calculates J and the time derivative of J.
// J is the Jacobian of the constraint function.
// Work in progress.

class Constraint {
public:
    static constexpr int MaxConstraintCount = 3;
    static constexpr int MaxBodyCount = 2;

    struct Output {
        double C[MaxConstraintCount];
        double J[MaxConstraintCount][3 * MaxBodyCount];
        double J_dot[MaxConstraintCount][3 * MaxBodyCount];
        double v_bias[MaxConstraintCount];
        double limits[MaxConstraintCount][2];
        double ks[MaxConstraintCount];
        double kd[MaxConstraintCount];
    };

public:
    Constraint(int constraintCount, int bodyCount);
    virtual ~Constraint();

    virtual void calculate(Output* output, SystemState* state);
    __forceinline int getConstraintCount() const { return m_constraintCount; }

    int m_index;
    int m_bodyCount;
    RigidBody* m_bodies[MaxBodyCount];

    double F_x[MaxConstraintCount][MaxBodyCount];
    double F_y[MaxConstraintCount][MaxBodyCount];
    double F_t[MaxConstraintCount][MaxBodyCount];

protected:
    inline void noLimits(Output* output)
    {
        for (int i = 0; i < MaxConstraintCount; ++i) {
            output->limits[i][0] = -DBL_MAX;
            output->limits[i][1] = DBL_MAX;
        }
    }

    int m_constraintCount;
};