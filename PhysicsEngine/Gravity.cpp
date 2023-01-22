#include "Gravity.hpp"

Gravity::Gravity() { mG = 9.81; }

Gravity::~Gravity() {}

void Gravity::apply(SystemState* state)
{
    const int n = state->n;
    for (int i = 0; i < n; ++i) state->f_y[i] += -state->m[i] * mG;
}