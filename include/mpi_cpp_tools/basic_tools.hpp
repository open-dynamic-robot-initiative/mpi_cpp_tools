/**
 * @file basic_tools.hpp
 * @author Manuel Wuthrich
 * @license License BSD-3-Clause
 * @copyright Copyright (c) 2019, New York University and Max Planck Gesellschaft.
 * @date 2019-08-05
 */

#pragma once

#include <cmath>


namespace mct
{

class NonnegDouble
{
public:
    NonnegDouble()
    {
        value_ = 0;
    }


    NonnegDouble(double value)
    {
        if(!std::isnan(value) && value < 0.0)
            throw std::invalid_argument("expected nonnegative double");
        value_ = value;
    }

    operator double() const
    {
        return value_;
    }

private:
    double value_;
};






}
