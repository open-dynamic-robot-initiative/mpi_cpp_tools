#pragma once

#include <cmath>
#include <math.h>
#include <Eigen/Eigen>
#include <limits>


#include "mpi_cpp_tools/basic_tools.hpp"
#include "mpi_cpp_tools/math.hpp"


namespace mct
{


union myUnion {
    double dValue;
    uint64_t iValue;
};




class LinearDynamics
{
public:
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1, 0, 10> Vector;

    LinearDynamics(Eigen::Vector4d parameters): LinearDynamics(parameters[0],
                                                parameters[1],
                                                parameters[2],
                                                parameters[3]) { }


    LinearDynamics(double jerk,
                   double initial_acceleration,
                   double initial_velocity,
                   double initial_position)
    {
        jerk_ = jerk;
        initial_acceleration_ = initial_acceleration;
        initial_velocity_ = initial_velocity;
        initial_position_= initial_position;
    }
    double get_acceleration(double t) const
    {
        return  jerk_ * t +
                initial_acceleration_;
    }
    double get_velocity(double t) const
    {
        return jerk_ * 0.5 * pow(t,2) +
                initial_acceleration_ * t +
                initial_velocity_;
    }
    double get_position(double t) const
    {

        std::cout.precision(std::numeric_limits<double>::max_digits10 + 2);





        double pow3 = t * t * t;
        double pow2 = t * t;


        myUnion bla;
        bla.iValue = 4619552210890228616;
        std::cout << "computing garbage " << std::endl;
        double garbage_pow = std::pow(bla.dValue, 3.01);
        std::cout << "done computing garbage " << bla.iValue << std::endl;


        std::cout << "computing position " << std::endl;


        double position = jerk_ * 0.5 * (1./3.) * pow3 +
                initial_acceleration_ * 0.5 * pow2 +
                initial_velocity_ * t +
                initial_position_;


        std::cout << "done computing position " << std::endl;


        return position;
    }

    Vector find_t_given_velocity(double velocity) const
    {
        double a = jerk_ * 0.5;
        double b = initial_acceleration_;
        double c = initial_velocity_ - velocity;

        double determinant = pow(b, 2) - 4 * a * c;

        Vector solutions(0);
        if(a == 0)
        {
            if(b != 0)
            {
                solutions.resize(1);
                solutions[0] = - c / b;
            }

        }
        else if(determinant == 0)
        {
            solutions.resize(1);
            solutions[0] = -b / 2 / a;
        }
        else if(determinant > 0)
        {
            double determinant_sqrt = std::sqrt(determinant);
            solutions.resize(2);
            solutions[0] = (-b + determinant_sqrt) / 2 / a;
            solutions[1] = (-b - determinant_sqrt) / 2 / a;
        }

        Vector positive_solutions(0);
        for(size_t i = 0; i < solutions.size(); i++)
        {
            if(solutions[i] >= 0)
            {
                mct::append_to_vector(positive_solutions, solutions[i]);
            }
        }

        return positive_solutions;
    }

protected:
    double jerk_;
    double initial_acceleration_;
    double initial_velocity_;
    double initial_position_;
};






class LinearDynamicsWithAccelerationConstraint: public LinearDynamics
{
public:

    void print_parameters() const
    {
        std::cout << "-------------------------------------------" << std::endl;
        std::cout << "jerk: " << jerk_ << std::endl
                  << "initial_acceleration: " << initial_acceleration_ << std::endl
                  << "initial_velocity: " << initial_velocity_ << std::endl
                  << "initial_position: " << initial_position_ << std::endl
                  << "acceleration_limit: " << acceleration_limit_ << std::endl
                  << "jerk_duration: " << jerk_duration_ << std::endl;
        std::cout << "-------------------------------------------" << std::endl;
    }

    typedef LinearDynamics::Vector Vector;

    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, 0, 10, 10>
    Matrix;


    LinearDynamicsWithAccelerationConstraint(Eigen::Matrix<double, 5, 1> parameters):
        LinearDynamicsWithAccelerationConstraint(parameters[0],
        parameters[1],
        parameters[2],
        parameters[3],
        parameters[4]) { }

    LinearDynamicsWithAccelerationConstraint(double jerk,
                                             double initial_acceleration,
                                             double initial_velocity,
                                             double initial_position,
                                             double abs_acceleration_limit):
        LinearDynamics(jerk,
                       initial_acceleration,
                       initial_velocity,
                       initial_position)
    {
        if(jerk_ > 0)
            acceleration_limit_ = abs_acceleration_limit;
        else
            acceleration_limit_ = -abs_acceleration_limit;

        set_initial_acceleration(initial_acceleration);
    }


    void set_initial_acceleration(double initial_acceleration)
    {
        if(std::fabs(initial_acceleration) > std::fabs(acceleration_limit_))
            throw std::invalid_argument("expected "
                                        "std::fabs(initial_acceleration) > "
                                        "abs_acceleration_limit");
        initial_acceleration_ = initial_acceleration;
        jerk_duration_ =
                (acceleration_limit_ - initial_acceleration_) / jerk_;
    }

    double get_acceleration(double t) const
    {
        if(t < jerk_duration_)
        {
            return  LinearDynamics::get_acceleration(t);
        }
        else
        {
            return acceleration_limit_;
        }

    }
    double get_velocity(double t) const
    {
        if(t < jerk_duration_)
        {
            return LinearDynamics::get_velocity(t);
        }
        else
        {
            return LinearDynamics::get_velocity(jerk_duration_) +
                    acceleration_limit_ * (t - jerk_duration_);
        }
    }
    double get_position(double t) const
    {
        if(t < jerk_duration_)
        {
            return LinearDynamics::get_position(t);
        }
        else
        {
            return LinearDynamics::get_position(jerk_duration_) +
                    LinearDynamics::get_velocity(jerk_duration_) * (t - jerk_duration_) +
                    acceleration_limit_ * 0.5 * pow(t - jerk_duration_, 2);
        }
    }

//    template<typename Array>
    Vector get_positions(const Vector& times) const
    {

//        std::cout << "beginning ---------------" << std::endl;
//        std::cout << times.size() << std::endl;


//        std::cout << "type: " << typeid(times).name() << std::endl;

//        std::cout << times << std::endl;
//        std::cout << "done printing ---------------" << std::endl;

        Vector positions(times.size());


//        std::cout << "printing positions ---------------" << std::endl;
//        std::cout << positions << std::endl;
//        std::cout << "done positions ---------------" << std::endl;


//        positions.resize(times.size());
        for(size_t i = 0; i < times.size(); i++)
        {
//            std::cout << "getting time " << std::endl;
            double time = times[i];

//            std::cout << "getting position " << std::endl;



            double position = get_position(time);
//            std::cout << "setting positions" << std::endl;

//            std::cout << "setting position " << std::endl;

            positions[i] = position;

//            std::cout << "done " << std::endl;

//            std::cout << "done setting positions" << std::endl;

        }

//        std::cout << "end ---------------" << std::endl;

        return positions;
    }

    Vector find_t_given_velocity(double velocity) const
    {
        Vector potential_solutions =
                LinearDynamics::find_t_given_velocity(velocity);

        Vector solutions(0);
        for(size_t i = 0; i < potential_solutions.size(); i++)
        {
            if(potential_solutions[i] <= jerk_duration_)
            {
                mct::append_to_vector(solutions, potential_solutions[i]);
            }
        }

        double potential_solution = jerk_duration_
                + (velocity - LinearDynamics::get_velocity(jerk_duration_))
                / acceleration_limit_;
        if(potential_solution > jerk_duration_ &&
                !(mct::contains(solutions, jerk_duration_) &&
                  mct::approx_equal(potential_solution, jerk_duration_)))
        {
            mct::append_to_vector(solutions, potential_solution);
        }



        if(solutions.size() > 2)
        {
            std::cout << "too many solutions, something went wrong!!!"
                      << std::endl;
            print_parameters();

            std::cout << "potential_solutions[0]: " << potential_solutions[0] << std::endl;

            std::cout << "potential_solutions size: " << potential_solutions.size() << " content: " << potential_solutions.transpose() << std::endl;


            std::cout << "solutions size: " << solutions.size() << " content: " << solutions.transpose() << std::endl;
            exit(-1);
        }
        return solutions;
    }

    bool will_exceed_jointly(const double& max_velocity,
                             const double& max_position) const
    {
        double certificate_time;
        return will_exceed_jointly(max_velocity, max_position, certificate_time);
    }


    bool will_exceed_jointly(const double& max_velocity,
                             const double& max_position,
                             double& certificate_time) const
    {


        Vector times = Vector::Zero(1);

        Vector dummy2;// = get_positions(Vector::Zero(1));


        Vector positions(times.size());

        for(size_t i = 0; i < times.size(); i++)
        {
            double time = times[i];
            double position = get_position(0);
            positions[i] = position;
        }




        Vector dummy;

        return false;
    }


    bool will_deceed_jointly(const double& min_velocity,
                             const double& min_position) const
    {
        double certificate_time;
        return will_deceed_jointly(min_velocity, min_position, certificate_time);
    }

    bool will_deceed_jointly(const double& min_velocity,
                             const double& min_position,
                             double& certificate_time) const
    {
        LinearDynamicsWithAccelerationConstraint
                flipped_dynamics(-jerk_,
                                 -initial_acceleration_,
                                 -initial_velocity_,
                                 -initial_position_,
                                 std::fabs(acceleration_limit_));

        return flipped_dynamics.will_exceed_jointly(-min_velocity,
                                                    -min_position,
                                                    certificate_time);
    }


private:
    double acceleration_limit_;
    double jerk_duration_;
};



double find_max_admissible_acceleration(
        const double& initial_velocity,
        const double& initial_position,
        const double& max_velocity,
        const double& max_position,
        const double& abs_jerk_limit,
        const double& abs_acceleration_limit)
{


    LinearDynamicsWithAccelerationConstraint dynamics(-1,
                                                      45,
                                                      -1,
                                                      -1,
                                                      90);
    dynamics.will_exceed_jointly(6, -std::numeric_limits<double>::infinity());


    double dummy[4];
}



double find_min_admissible_acceleration(
        const double& initial_velocity,
        const double& initial_position,
        const double& min_velocity,
        const double& min_position,
        const double& abs_jerk_limit,
        const double& abs_acceleration_limit)
{
    double dummy;
    find_max_admissible_acceleration(0, 0, 0, 0, 0, 0);
}



class SafetyConstraint
{
public:
    SafetyConstraint()
    {
        min_velocity_ = -std::numeric_limits<double>::infinity();
        min_position_ = -std::numeric_limits<double>::infinity();
        max_velocity_ = std::numeric_limits<double>::infinity();
        max_position_ = std::numeric_limits<double>::infinity();
        max_torque_ = 1.0;
        max_jerk_ = 1.0;
        inertia_ = 1.0;
    }

    SafetyConstraint(double min_velocity,
                     double min_position,
                     double max_velocity,
                     double max_position,
                     double max_torque,
                     double max_jerk,
                     double inertia)
    {
        min_velocity_ = min_velocity;
        min_position_ = min_position;
        max_velocity_ = max_velocity;
        max_position_ = max_position;
        max_torque_ = max_torque;
        max_jerk_ = max_jerk;
        inertia_ = inertia;
    }


    double get_safe_torque(const double& torque,
                           const double& velocity,
                           const double& position)
    {


//        double max_achievable_acc = max_torque_ / inertia_;



       find_min_admissible_acceleration(velocity,
                                                 position,
                                                 min_velocity_,
                                                 min_position_,
                                                 max_jerk_,
                                                 max_torque_ / inertia_);



    }

    double min_velocity_;
    double min_position_;
    double max_velocity_;
    double max_position_;
    double max_torque_;
    double max_jerk_;
    double inertia_;
};



}
