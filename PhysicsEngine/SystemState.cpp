#include "SystemState.hpp"

SystemState::SystemState()
{
    indexMap = nullptr;

    angularAcceleration = angularVelocity = orientation = nullptr;

    accX = accY = nullptr;
    velX = velY = nullptr;
    posX = posY = nullptr;

    f_x = f_y = nullptr;
    t = nullptr;

    m = nullptr;

    r_x = r_y = r_t = 0;

    n = n_c = dt = 0.0;
}

SystemState::~SystemState()
{
    assert(n == 0);
    assert(n_c == 0);
}

void SystemState::copy(const SystemState* state)
{
    resize(state->n, state->n_c);

    if (state->n == 0) return;

    memcpy((void*)indexMap, (void*)state->indexMap, sizeof(int) * n_c);

    memcpy((void*)angularAcceleration, (void*)state->angularAcceleration, sizeof(double) * n);
    memcpy((void*)angularVelocity, (void*)state->angularVelocity, sizeof(double) * n);
    memcpy((void*)orientation, (void*)state->orientation, sizeof(double) * n);

    memcpy((void*)accX, (void*)state->accX, sizeof(double) * n);
    memcpy((void*)accY, (void*)state->accY, sizeof(double) * n);
    memcpy((void*)velX, (void*)state->velX, sizeof(double) * n);
    memcpy((void*)velY, (void*)state->velY, sizeof(double) * n);
    memcpy((void*)posX, (void*)state->posX, sizeof(double) * n);
    memcpy((void*)posY, (void*)state->posY, sizeof(double) * n);

    memcpy((void*)f_x, (void*)state->f_x, sizeof(double) * n);
    memcpy((void*)f_y, (void*)state->f_y, sizeof(double) * n);
    memcpy((void*)t, (void*)state->t, sizeof(double) * n);

    memcpy((void*)m, (void*)state->m, sizeof(double) * n);

    memcpy((void*)r_x, (void*)state->r_x, sizeof(double) * n_c * 2);
    memcpy((void*)r_y, (void*)state->r_y, sizeof(double) * n_c * 2);
    memcpy((void*)r_t, (void*)state->r_t, sizeof(double) * n_c * 2);
}

void SystemState::resize(int bodyCount, int constraintCount)
{
    if (n >= bodyCount && n_c >= constraintCount) return;

    destroy();

    n = bodyCount;
    n_c = constraintCount;

    indexMap = new int[n_c];

    angularAcceleration = new double[n];
    angularVelocity = new double[n];
    orientation = new double[n];

    accX = new double[n];
    accY = new double[n];
    velX = new double[n];
    velY = new double[n];
    posX = new double[n];
    posY = new double[n];

    f_x = new double[n];
    f_y = new double[n];
    t = new double[n];

    m = new double[n];

    r_x = new double[(size_t)n_c * 2];
    r_y = new double[(size_t)n_c * 2];
    r_t = new double[(size_t)n_c * 2];
}

void SystemState::destroy()
{
    if (n > 0) {
        freeArray(angularAcceleration);
        freeArray(angularVelocity);
        freeArray(orientation);

        freeArray(accX);
        freeArray(accY);
        freeArray(velX);
        freeArray(velY);
        freeArray(posX);
        freeArray(posY);

        freeArray(f_x);
        freeArray(f_y);
        freeArray(t);

        freeArray(m);
    }

    if (n_c > 0) {
        freeArray(indexMap);

        freeArray(r_x);
        freeArray(r_y);
        freeArray(r_t);
    }

    n = 0;
    n_c = 0;
}

void SystemState::localToWorld(double x, double y, double* x_t, double* y_t, int body)
{
    const double x0 = posX[body];
    const double y0 = posY[body];
    const double orientation = this->orientation[body];

    const double cosTheta = cos(orientation);
    const double sinTheta = sin(orientation);

    *x_t = cosTheta * x - sinTheta * y + x0;
    *y_t = sinTheta * x + cosTheta * y + y0;
}

void SystemState::velocityAtPoint(double x, double y, double* velX, double* velY, int body)
{
    double w_x, w_y;
    localToWorld(x, y, &w_x, &w_y, body);

    const double angularVelocity = this->angularVelocity[body];
    const double angularToLinear_x = -angularVelocity * (w_y - this->posY[body]);
    const double angularToLinear_y = angularVelocity * (w_x - this->posX[body]);

    *velX = this->velX[body] + angularToLinear_x;
    *velY = this->velY[body] + angularToLinear_y;
}

void SystemState::applyForce(double x_l, double y_l, double f_x, double f_y, int body)
{
    double w_x, w_y;
    localToWorld(x_l, y_l, &w_x, &w_y, body);

    this->f_x[body] += f_x;
    this->f_y[body] += f_y;

    this->t[body] += (w_y - this->posY[body]) * -f_x + (w_x - this->posX[body]) * f_y;
}

void freeArray(double*& data)
{
    delete[] data;
    data = nullptr;
}

void freeArray(int*& data)
{
    delete[] data;
    data = nullptr;
}