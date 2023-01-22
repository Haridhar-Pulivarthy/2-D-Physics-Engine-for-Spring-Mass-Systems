#include "StaticForces.hpp"

StaticForces::StaticForces()
{
    m_forX = m_forY = 0.0;
    m_posX = m_posY = 0.0;
    m_body = nullptr;
}

StaticForces::~StaticForces() {}

void StaticForces::apply(SystemState* state)
{
    state->applyForce(m_posX, m_posY, m_forX, m_forY, m_body->index);
}

void StaticForces::setForce(double forX, double forY)
{
    m_forX = forX;
    m_forY = forY;
}

void StaticForces::setPosition(double posX, double posY)
{
    m_posX = posX;
    m_posY = posY;
}