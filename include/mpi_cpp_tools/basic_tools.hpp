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
