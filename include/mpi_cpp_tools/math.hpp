/**
 * @file math.hpp
 * @author Manuel Wuthrich
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-08-05
 */

#pragma once

#include <cmath>
#include <algorithm>

namespace mct
{

double clamp(const double &value, const double &limit_a, const double &limit_b)
{
    if (limit_b > limit_a)
        return std::max(limit_a, std::min(value, limit_b));

    return std::max(limit_b, std::min(value, limit_a));
}

template <typename Vector>
Vector clamp(const Vector &vector,
             const double &limit_a, const double &limit_b)
{
    Vector clamped_vector = vector;

    for(size_t i = 0; i < size_t(clamped_vector.size()); i++)
    {
        clamped_vector[i] = clamp(clamped_vector[i], limit_a, limit_b);
    }
    return clamped_vector;
}

template <typename Vector>
void append_to_vector(Vector &vector,
                      const double &element)
{
    vector.conservativeResize(vector.size() + 1);
    vector[vector.size() - 1] = element;
}

template <typename Matrix>
void append_rows_to_matrix(Matrix &matrix,
                           const Matrix &rows)
{
    if (matrix.cols() != rows.cols())
        throw std::invalid_argument("need to have same number of cols");

    matrix.conservativeResize(matrix.rows() + rows.rows(), matrix.cols());
    matrix.bottomRows(rows.rows()) = rows;
}

bool approx_equal(double x, double y, double epsilon = 1e-10)
{
    return (std::fabs(x - y) < epsilon);
}

template <typename Vector>
bool contains(Vector v, double x)
{
    for (size_t i = 0; i < size_t(v.size()); i++)
    {
        if (mct::approx_equal(v[i], x))
        {
            return true;
        }
    }
    return false;
}

} // namespace mct
