#include "Matrix.hpp"

Matrix::Matrix()
{
    m_matrix = nullptr;
    m_data = nullptr;
    m_width = m_height = 0;
    m_capacityWidth = m_capacityHeight = 0;
}

Matrix::Matrix(int width, int height, double value)
{
    m_matrix = nullptr;
    m_data = nullptr;
    m_width = m_height = 0;
    m_capacityWidth = m_capacityHeight = 0;

    initialize(width, height, value);
}

Matrix::~Matrix() { assert(m_matrix == nullptr); }

void Matrix::initialize(int width, int height, double value)
{
    resize(width, height);

    for (int i = 0; i < height; ++i)
        for (int j = 0; j < width; ++j)
            m_matrix[i][j] = value;
}

void Matrix::initialize(int width, int height)
{
    resize(width, height);
    memset(m_data, 0, sizeof(double) * width * height);
}

void Matrix::resize(int width, int height)
{
    if (width == m_width && height == m_height) return;

    else if (width > m_capacityWidth || height > m_capacityHeight)
    {
        destroy();

        m_capacityWidth = (width > m_capacityWidth) ? width : m_capacityWidth;
        m_capacityHeight = (height > m_capacityHeight) ? height : m_capacityHeight;

        m_data = new double[(size_t)m_capacityWidth * m_capacityHeight];
        m_matrix = new double* [m_capacityHeight];
    }

    m_height = height;
    m_width = width;

    for (int i = 0; i < height; ++i) m_matrix[i] = &m_data[i * width];
}

void Matrix::destroy()
{
    if (m_matrix == nullptr) return;

    delete[] m_matrix;
    delete[] m_data;

    m_matrix = nullptr;
    m_data = nullptr;

    m_width = m_height = 0;
    m_capacityWidth = m_capacityHeight = 0;
}

void Matrix::set(const double* data) { memcpy(m_data, data, sizeof(double) * m_width * m_height); }

void Matrix::set(Matrix* reference)
{
    resize(reference->m_width, reference->m_height);

    for (int i = 0; i < reference->m_height; ++i)
        for (int j = 0; j < reference->m_width; ++j)
            m_matrix[i][j] = reference->m_matrix[i][j];
}

void Matrix::multiply(Matrix& b, Matrix* target)
{
    assert(m_width == b.m_height);

    target->resize(b.m_width, m_height);

    for (int i = 0; i < m_height; ++i)
    {
        for (int j = 0; j < b.m_width; ++j)
        {
            double v = 0.0;
            for (int ii = 0; ii < m_width; ++ii)
                v += m_matrix[i][ii] * b.m_matrix[ii][j];
            target->m_matrix[i][j] = v;
        }
    }
}

void Matrix::componentMultiply(Matrix& b, Matrix* target)
{
    assert(m_height == b.m_height);
    assert(m_width == b.m_width);

    target->resize(m_width, m_height);

    for (int i = 0; i < m_height; ++i)
        for (int j = 0; j < m_width; ++j)
            target->set(j, i, get(j, i) * b.get(j, i));
}

void Matrix::transposeMultiply(Matrix& b, Matrix* target)
{
    assert(m_height == b.m_height);

    target->resize(b.m_width, m_width);

    for (int i = 0; i < m_width; ++i)
    {
        for (int j = 0; j < b.m_width; ++j)
        {
            double v = 0.0;
            for (int ii = 0; ii < m_height; ++ii)
                v += m_matrix[ii][i] * b.m_matrix[ii][j];
            target->m_matrix[i][j] = v;
        }
    }
}

void Matrix::leftScale(Matrix& scale, Matrix* target)
{
    assert(scale.m_width == 1);
    assert(scale.m_height == m_height);

    target->resize(m_width, m_height);

    for (int i = 0; i < m_height; ++i)
        for (int j = 0; j < m_width; ++j)
            target->m_matrix[i][j] = scale.m_matrix[i][0] * m_matrix[i][j];
}

void Matrix::rightScale(Matrix& scale, Matrix* target)
{
    assert(scale.m_width == 1);
    assert(scale.m_height == m_width);

    target->resize(m_width, m_height);

    for (int i = 0; i < m_height; ++i)
        for (int j = 0; j < m_width; ++j)
            target->m_matrix[i][j] = scale.m_matrix[j][0] * m_matrix[i][j];
}

void Matrix::scale(double s, Matrix* target)
{
    target->resize(m_width, m_height);

    for (int i = 0; i < m_height; ++i)
        for (int j = 0; j < m_width; ++j)
            target->m_matrix[i][j] = s * m_matrix[i][j];
}

void Matrix::subtract(Matrix& b, Matrix* target)
{
    assert(b.m_width == m_width);
    assert(b.m_height == m_height);

    target->resize(m_width, m_height);

    for (int i = 0; i < m_height; ++i)
        for (int j = 0; j < m_width; ++j)
            target->m_matrix[i][j] = m_matrix[i][j] - b.m_matrix[i][j];
}

void Matrix::add(Matrix& b, Matrix* target)
{
    assert(b.m_width == m_width);
    assert(b.m_height == m_height);

    target->resize(m_width, m_height);

    for (int i = 0; i < m_height; ++i)
        for (int j = 0; j < m_width; ++j)
            target->m_matrix[i][j] = m_matrix[i][j] + b.m_matrix[i][j];
}

void Matrix::negate(Matrix* target)
{
    target->resize(m_width, m_height);

    for (int i = 0; i < m_height; ++i)
        for (int j = 0; j < m_width; ++j)
            target->m_matrix[i][j] = -m_matrix[i][j];
}

bool Matrix::equals(Matrix& b, double err)
{
    if (getWidth() != b.getWidth()) return false;
    if (getHeight() != b.getHeight()) return false;

    for (int i = 0; i < getHeight(); ++i)
        for (int j = 0; j < getWidth(); ++j)
            if (abs(get(j, i) - b.get(j, i)) > err) return false;
    return true;
}

double Matrix::vectorMagnitudeSquared() const
{
    assert(m_width == 1);

    double mag = 0;
    for (int i = 0; i < m_height; ++i) mag += m_matrix[0][i] * m_matrix[0][i];
    return mag;
}

double Matrix::dot(Matrix& b) const
{
    assert(m_width == 1);
    assert(b.m_width == 1);
    assert(b.m_height == m_height);

    double result = 0;
    for (int i = 0; i < m_height; ++i) result += m_matrix[0][i] * b.m_matrix[0][i];
    return result;
}

void Matrix::madd(Matrix& b, double s)
{
    assert(m_width == b.m_width);
    assert(m_height == b.m_height);

    for (int i = 0; i < m_height; ++i)
        for (int j = 0; j < m_width; ++j)
            m_matrix[i][j] += b.m_matrix[i][j] * s;
}

void Matrix::pmadd(Matrix& b, double s)
{
    assert(m_width == b.m_width);
    assert(m_height == b.m_height);

    for (int i = 0; i < m_height; ++i)
        for (int j = 0; j < m_width; ++j)
            m_matrix[i][j] = s * m_matrix[i][j] + b.m_matrix[i][j];
}

void Matrix::transpose(Matrix* target)
{
    target->resize(m_height, m_width);

    for (int i = 0; i < m_width; ++i)
        for (int j = 0; j < m_height; ++j)
            target->m_matrix[i][j] = m_matrix[j][i];
}