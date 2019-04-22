
#include <eigen3/Eigen/Core>
#include <gtest/gtest.h>

#include <math.h>
#include <Eigen/Dense>


#include "mpi_cpp_tools/basic_tools.hpp"
#include "mpi_cpp_tools/math.hpp"
#include "mpi_cpp_tools/dynamical_systems.hpp"










//Thread 6 "demo_finger_pos" received signal SIGSEGV, Segmentation fault.
//[Switching to Thread 0x7ffff7fe5700 (LWP 6202)]
//__mul (x=x@entry=0x7ffff7fe33c0, y=y@entry=0x7ffff7fe3270, z=z@entry=0x7ffff7fe3120, p=p@entry=32)
//    at ../sysdeps/ieee754/dbl-64/mpa.c:682
//682	../sysdeps/ieee754/dbl-64/mpa.c: No such file or directory.
//(gdb) backtrace
//#0  __mul (x=x@entry=0x7ffff7fe33c0, y=y@entry=0x7ffff7fe3270, z=z@entry=0x7ffff7fe3120, p=p@entry=32)
//    at ../sysdeps/ieee754/dbl-64/mpa.c:682
//#1  0x00007ffff5b4a495 in __inv (p=32, y=0x7ffff7fe3120, x=0x7ffff7fe36c0) at ../sysdeps/ieee754/dbl-64/mpa.c:882
//#2  __dvd (x=x@entry=0x7ffff7fe3960, y=y@entry=0x7ffff7fe36c0, z=z@entry=0x7ffff7fe3810, p=p@entry=32)
//    at ../sysdeps/ieee754/dbl-64/mpa.c:903
//#3  0x00007ffff5b4aad1 in __mpexp (x=x@entry=0x7ffff7fe3b00, y=y@entry=0x7ffff7fe3c50, p=p@entry=32)
//    at ../sysdeps/ieee754/dbl-64/mpexp.c:142
//#4  0x00007ffff5b4adc6 in __mplog (x=x@entry=0x7ffff7fe3e10, y=y@entry=0x7ffff7fe40b0, p=p@entry=32)
//    at ../sysdeps/ieee754/dbl-64/mplog.c:59
//#5  0x00007ffff5b4c119 in __slowpow (x=x@entry=0.0034326188580546813, y=y@entry=3, z=-5.6744317936661055)
//    at ../sysdeps/ieee754/dbl-64/slowpow.c:115
//#6  0x00007ffff5b0c54c in power1 (y=3, x=0.0034326188580546813) at ../sysdeps/ieee754/dbl-64/e_pow.c:239
//#7  __ieee754_pow_sse2 (x=<optimized out>, y=<optimized out>) at ../sysdeps/ieee754/dbl-64/e_pow.c:115
//#8  0x00007ffff5b1be54 in __pow (x=0.0034326188580546813, y=3) at w_pow.c:27
//#9  0x0000000000444023 in mct::LinearDynamics::get_position (this=0x7ffff7fe4b40, t=...)
//    at /home/manuel/devel/workspace/src/catkin/tools/mpi_cpp_tools/include/mpi_cpp_tools/dynamical_systems.hpp:53
//#10 0x00000000004447d9 in mct::LinearDynamicsWithAccelerationConstraint::get_position (this=0x7ffff7fe4b40, t=...)
//    at /home/manuel/devel/workspace/src/catkin/tools/mpi_cpp_tools/include/mpi_cpp_tools/dynamical_systems.hpp:200
//#11 0x00000000004480e8 in mct::LinearDynamicsWithAccelerationConstraint::get_positions<Eigen::Matrix<double, -1, 1, 0, 10, 1> > (
//    this=0x7ffff7fe4b40, times=...)
//    at /home/manuel/devel/workspace/src/catkin/tools/mpi_cpp_tools/include/mpi_cpp_tools/dynamical_systems.hpp:217
//#12 0x0000000000444ed3 in mct::LinearDynamicsWithAccelerationConstraint::will_exceed_jointly (this=0x7ffff7fe4b40,
//    max_velocity=@0x7ffff7fe4bd8: 6, max_position=@0x7ffff7fe4be0: -inf, certificate_time=@0x7ffff7fe4ac0: 6.9533491710994045e-310)
//    at /home/manuel/devel/workspace/src/catkin/tools/mpi_cpp_tools/include/mpi_cpp_tools/dynamical_systems.hpp:316
//#13 0x0000000000444ce9 in mct::LinearDynamicsWithAccelerationConstraint::will_exceed_jointly (this=0x7ffff7fe4b40,
//    max_velocity=@0x7ffff7fe4bd8: 6, max_position=@0x7ffff7fe4be0: -inf)
//    at /home/manuel/devel/workspace/src/catkin/tools/mpi_cpp_tools/include/mpi_cpp_tools/dynamical_systems.hpp:271
//#14 0x0000000000443066 in mct::find_max_admissible_acceleration (initial_velocity=@0x7ffff7fe4bc8: -0.011760214219915013,
//    initial_position=@0x7ffff7fe4bd0: -0.34909119204111372, max_velocity=@0x7ffff7fe4bd8: 6, max_position=@0x7ffff7fe4be0: -inf,
//    abs_jerk_limit=..., abs_acceleration_limit=...)
//    at /home/manuel/devel/workspace/src/catkin/tools/mpi_cpp_tools/include/mpi_cpp_tools/dynamical_systems.hpp:417
//#15 0x000000000044315b in mct::find_min_admissible_acceleration (initial_velocity=@0x7ffff7fe4cb0: 0.011760214219915013,
//    initial_position=@0x7ffff7fe4cd0: 0.34909119204111372, min_velocity=@0x779cb0: -6, min_position=@0x779cb8: inf, abs_jerk_limit=...,
//    abs_acceleration_limit=...)
//    at /home/manuel/devel/workspace/src/catkin/tools/mpi_cpp_tools/include/mpi_cpp_tools/dynamical_systems.hpp:439
//#16 0x00000000004452d2 in mct::SafetyConstraint::get_safe_torque (this=0x779cb0, torque=@0x7ffff7fe4e80: 0.16086307155422794,
//    velocity=@0x7ffff7fe4cb0: 0.011760214219915013, position=@0x7ffff7fe4cd0: 0.34909119204111372)
//    at /home/manuel/devel/workspace/src/catkin/tools/mpi_cpp_tools/include/mpi_cpp_tools/dynamical_systems.hpp:525
//#17 0x0000000000445b1e in robot_interfaces::Finger::constrain_torques (this=0x779c90, desired_torques=...)
//    at /home/manuel/devel/workspace/src/catkin/robots/robot_interfaces/include/robot_interfaces/finger.hpp:91
//#18 0x00000000004458ec in robot_interfaces::Finger::constrain_and_apply_torques (this=0x779c90, desired_torques=...)
//    at /home/manuel/devel/workspace/src/catkin/robots/robot_interfaces/include/robot_interfaces/finger.hpp:60
//#19 0x0000000000443369 in control_loop (finger_and_sliders_void_ptr=0x7fffffffd640)
//    at /home/manuel/devel/workspace/src/catkin/robots/blmc_robots/demos/demo_finger_position_control.cpp:50
//#20 0x00007ffff75246ba in start_thread (arg=0x7ffff7fe5700) at pthread_create.c:333
//#21 0x00007ffff561f41d in clone () at ../sysdeps/unix/sysv/linux/x86_64/clone.S:109









//Thread 6 "demo_finger_pos" received signal SIGSEGV, Segmentation fault.
//[Switching to Thread 0x7ffff7fe5700 (LWP 24895)]
//do_lookup_x (undef_name=undef_name@entry=0x7ffff5afeb9c "memmove", new_hash=new_hash@entry=3184959259,
//    old_hash=old_hash@entry=0x7ffff7fe3120, ref=0x7ffff5afb868, result=result@entry=0x7ffff7fe3130, scope=0x7ffff7ffe420, i=0,
//    version=0x7ffff7fb0738, flags=5, skip=0x0, type_class=1, undef_map=0x7ffff7fb39e8) at dl-lookup.c:355
//355	dl-lookup.c: No such file or directory.
//(gdb) backtrace
//#0  do_lookup_x (undef_name=undef_name@entry=0x7ffff5afeb9c "memmove", new_hash=new_hash@entry=3184959259,
//    old_hash=old_hash@entry=0x7ffff7fe3120, ref=0x7ffff5afb868, result=result@entry=0x7ffff7fe3130, scope=0x7ffff7ffe420, i=0,
//    version=0x7ffff7fb0738, flags=5, skip=0x0, type_class=1, undef_map=0x7ffff7fb39e8) at dl-lookup.c:355
//#1  0x00007ffff7de1b1f in _dl_lookup_symbol_x (undef_name=0x7ffff5afeb9c "memmove", undef_map=0x7ffff7fb39e8,
//    ref=ref@entry=0x7ffff7fe31e8, symbol_scope=0x7ffff7fb3d40, version=0x7ffff7fb0738, type_class=type_class@entry=1, flags=5,
//    skip_map=0x0) at dl-lookup.c:829
//#2  0x00007ffff7de6ac6 in _dl_fixup (l=<optimized out>, reloc_arg=<optimized out>) at ../elf/dl-runtime.c:111
//#3  0x00007ffff7dee337 in _dl_runtime_resolve_avx512 () at ../sysdeps/x86_64/dl-trampoline.h:112
//#4  0x00007ffff5568106 in hack_digit (p=0x0) at printf_fp.c:186
//#5  0x00007ffff5569919 in __GI___printf_fp_l (fp=<optimized out>, loc=0x0, info=<optimized out>, args=<optimized out>)
//    at printf_fp.c:1182
//#6  0x00007ffff556592d in _IO_vfprintf_internal (s=0x7ffff7fe4730, format=<optimized out>, ap=0x7ffff7fe4890) at vfprintf.c:1663
//#7  0x0000000000449df6 in Eigen::DenseCoeffsBase<Eigen::Matrix<double, -1, 1, 0, 10, 1>, 1>::coeffRef (this=0x7ffff7fe49b0, index=1)
//    at /usr/include/eigen3/Eigen/src/Core/DenseCoeffsBase.h:380
//#8  0x00007ffff5b0c54c in power1 (y=3, x=4.1273982640614797) at ../sysdeps/ieee754/dbl-64/e_pow.c:239
//#9  __ieee754_pow_sse2 (x=<optimized out>, y=<optimized out>) at ../sysdeps/ieee754/dbl-64/e_pow.c:115
//#10 0x00007ffff5b1be54 in __pow (x=4.1273982640614797, y=3) at w_pow.c:27
//#11 0x0000000000444151 in mct::LinearDynamics::get_position (this=0x7ffff7fe4b40, t=...)
//    at /home/manuel/devel/workspace/src/catkin/tools/mpi_cpp_tools/include/mpi_cpp_tools/dynamical_systems.hpp:55
//#12 0x0000000000444907 in mct::LinearDynamicsWithAccelerationConstraint::get_position (this=0x7ffff7fe4b40, t=...)
//    at /home/manuel/devel/workspace/src/catkin/tools/mpi_cpp_tools/include/mpi_cpp_tools/dynamical_systems.hpp:203
//#13 0x0000000000448118 in mct::LinearDynamicsWithAccelerationConstraint::get_positions<Eigen::Matrix<double, -1, 1, 0, 10, 1> > (
//    this=0x7ffff7fe4b40, times=...)
//    at /home/manuel/devel/workspace/src/catkin/tools/mpi_cpp_tools/include/mpi_cpp_tools/dynamical_systems.hpp:219
//#14 0x00000000004450ca in mct::LinearDynamicsWithAccelerationConstraint::will_exceed_jointly (this=0x7ffff7fe4b40,
//    max_velocity=@0x7ffff7fe4bd8: 6, max_position=@0x7ffff7fe4be0: -inf, certificate_time=@0x7ffff7fe4ac0: 6.9533491710997998e-310)
//    at /home/manuel/devel/workspace/src/catkin/tools/mpi_cpp_tools/include/mpi_cpp_tools/dynamical_systems.hpp:335
//#15 0x0000000000444e17 in mct::LinearDynamicsWithAccelerationConstraint::will_exceed_jointly (this=0x7ffff7fe4b40,
//    max_velocity=@0x7ffff7fe4bd8: 6, max_position=@0x7ffff7fe4be0: -inf)
//    at /home/manuel/devel/workspace/src/catkin/tools/mpi_cpp_tools/include/mpi_cpp_tools/dynamical_systems.hpp:271
//#16 0x0000000000443178 in mct::find_max_admissible_acceleration (initial_velocity=@0x7ffff7fe4bc8: 0.0073236929977179045,
//    initial_position=@0x7ffff7fe4bd0: -0.0018132798072076906, max_velocity=@0x7ffff7fe4bd8: 6, max_position=@0x7ffff7fe4be0: -inf,
//    abs_jerk_limit=..., abs_acceleration_limit=...)
//    at /home/manuel/devel/workspace/src/catkin/tools/mpi_cpp_tools/include/mpi_cpp_tools/dynamical_systems.hpp:429
//#17 0x0000000000443289 in mct::find_min_admissible_acceleration (initial_velocity=@0x7ffff7fe4cb8: -0.0073236929977179045,
//    initial_position=@0x7ffff7fe4cd8: 0.0018132798072076906, min_velocity=@0x779ce8: -6, min_position=@0x779cf0: inf,
//    abs_jerk_limit=..., abs_acceleration_limit=...)
//    at /home/manuel/devel/workspace/src/catkin/tools/mpi_cpp_tools/include/mpi_cpp_tools/dynamical_systems.hpp:453
//#18 0x0000000000445400 in mct::SafetyConstraint::get_safe_torque (this=0x779ce8, torque=@0x7ffff7fe4e88: 0.087092558180534269,
//    velocity=@0x7ffff7fe4cb8: -0.0073236929977179045, position=@0x7ffff7fe4cd8: 0.0018132798072076906)
//    at /home/manuel/devel/workspace/src/catkin/tools/mpi_cpp_tools/include/mpi_cpp_tools/dynamical_systems.hpp:539
//#19 0x0000000000445b58 in robot_interfaces::Finger::constrain_torques (this=0x779c90, desired_torques=...)
//    at /home/manuel/devel/workspace/src/catkin/robots/robot_interfaces/include/robot_interfaces/finger.hpp:94
//#20 0x0000000000445926 in robot_interfaces::Finger::constrain_and_apply_torques (this=0x779c90, desired_torques=...)
//    at /home/manuel/devel/workspace/src/catkin/robots/robot_interfaces/include/robot_interfaces/finger.hpp:63
//#21 0x0000000000443497 in control_loop (finger_and_sliders_void_ptr=0x7fffffffd640)
//    at /home/manuel/devel/workspace/src/catkin/robots/blmc_robots/demos/demo_finger_position_control.cpp:50
//#22 0x00007ffff75246ba in start_thread (arg=0x7ffff7fe5700) at pthread_create.c:333
//#23 0x00007ffff561f41d in clone () at ../sysdeps/unix/sysv/linux/x86_64/clone.S:109









//beginning ---------------
//2
//4.516512182930473962
//4.600080224296088538
//done printing ---------------
//printing positions ---------------
//-3.696746394381638545
// 24.02211473689662213
//done positions ---------------
//getting positions 0
//setting positions
//done setting positions
//getting positions 1
//setting positions
//done setting positions
//end ---------------
//beginning ---------------
//2
//1.093996806853825277
//8.022252277618830973
//done printing ---------------
//printing positions ---------------
//9.911955730192348213
//10.41341261232263271
//done positions ---------------
//getting positions 0
//setting positions
//done setting positions
//getting positions 1
//setting positions
//done setting positions
//end ---------------
//beginning ---------------
//2
//4.544672142959446326
//4.571576941513209924
//done printing ---------------
//printing positions ---------------
//-3.696849113949709498
//  24.0165906977531769
//done positions ---------------
//getting positions 0




//beginning ---------------
//1
//type: N5Eigen6MatrixIdLin1ELi1ELi0ELi10ELi1EEE
//14.23476785735023498
//done printing ---------------
//printing positions ---------------
//28.06773752340831862
//done positions ---------------
//getting positions 0
//setting positions
//done setting positions
//end ---------------
//beginning ---------------
//2
//type: N5Eigen6MatrixIdLin1ELi1ELi0ELi10ELi1EEE
//3.582964988408914753
//10.47953501159108569
//done printing ---------------
//printing positions ---------------
//247.6415702402793784
//390.1756908910942911
//done positions ---------------
//getting positions 0
//setting positions
//done setting positions
//getting positions 1



//type: N5Eigen6MatrixIdLin1ELi1ELi0ELi10ELi1EEE
//0.5690740202762465394
// 7.505877151598753017
//done printing ---------------
//printing positions ---------------
//-1.906664200584287761
// 25.75587405991478107
//done positions ---------------
//getting positions 0
//setting positions
//done setting positions
//getting positions 1
//setting positions
//done setting positions
//end ---------------
//beginning ---------------
//2
//type: N5Eigen6MatrixIdLin1ELi1ELi0ELi10ELi1EEE
//3.864821536806196001
//4.210129635068803999
//done printing ---------------
//printing positions ---------------
//-1.905773321992936031
// 25.91036656420572726
//done positions ---------------
//getting positions 0
//setting positions
//done setting positions
//getting positions 1





// 4619552210890228616



using namespace mct;

double epsilon = 1e-10;


double sample_uniformely(const double& min, const double& max)
{



    if(min > max)
    {
        exit(-1);
    }
    double sample = min + double(rand()) / RAND_MAX * (max - min);
    return sample;
}

//TEST(SafetyConstraint, segfault)
//{
//    srand(0);

//    SafetyConstraint safety_constraint;
//    safety_constraint.min_velocity_ = -3.0;
//    safety_constraint.min_position_ = std::numeric_limits<double>::infinity();
//    safety_constraint.max_velocity_ = 3.0;
//    safety_constraint.max_position_ = -std::numeric_limits<double>::infinity();
//    safety_constraint.max_torque_ = 2.0 * 0.02 * 9 * 9.79;
//    std::cout << safety_constraint.max_torque_ << std::endl;
//    safety_constraint.inertia_ = 1.0; //dummy
////    safety_constraint.max_jerk_ = 1.0; // dummy


////    safety_constraint.get_safe_torque(-0.113588, -0.000622762, 0.031576);


//    for(size_t i = 0; i < 10000; i++)
//    {

//        double torque = sample_uniformely(-0.113589, -0.113587);
//        double velocity = sample_uniformely(-0.000622763, -0.000622761);
//        double position = sample_uniformely(0.031575, 0.031577);

//        double safe_torque = safety_constraint.get_safe_torque(torque, velocity, position);


//    }


//    for(size_t i = 0; i < 10000; i++)
//    {

//        double torque = sample_uniformely(-2, 2);
//        double velocity = sample_uniformely(-3.0, 3.0);
//        double position = sample_uniformely(-2, 2);



//        safety_constraint.get_safe_torque(torque, velocity, position);


//    }



//}

//x=0.0034326188580546813, y=3




//TEST(segfault, segfault)
//{
//    while(true)
//    {
//        myUnion bla;
//        bla.iValue = 4619552210890228616;
//        std::cout << "computing garbage " << std::endl;
//        double garbage_pow = std::pow(bla.dValue, 3.01);
//        std::cout << "done computing garbage " << bla.iValue << std::endl;

//    }
//}


TEST(pow, segfault)
{
    SafetyConstraint constraint;
    double max_torque_ = 2.0 * 0.02 * 9.0;

    constraint.min_velocity_ = -6.0;
    constraint.min_position_ = std::numeric_limits<double>::infinity();
    constraint.max_velocity_ = 6.0;
    constraint.max_position_ = -std::numeric_limits<double>::infinity();
    constraint.max_torque_ = max_torque_;
    constraint.inertia_ = 0.004;

    constraint.get_safe_torque(2, 0.0, 0.0);

    std::cout << "done!" << std::endl;
    exit(-1);
}

//TEST(LinearDynamicsWithAccelerationConstraint, segfault)
//{
//    LinearDynamicsWithAccelerationConstraint dynamics(-1,
//                                                      3.42773,
//                                                      -0.0117602,
//                                                      -0.677384,
//                                                      90);
//    dynamics.print_parameters();
//    dynamics.will_exceed_jointly(6, -std::numeric_limits<double>::infinity());


////            -------------------------------------------
////            jerk: -1
////        initial_acceleration: 3.42773
////        initial_velocity: -0.0117602
////        initial_position: -0.677384
////        acceleration_limit: -90
////        jerk_duration: 93.4277
////        -------------------------------------------
////        max_velocity: 6
////        max_position: -inf
////        Segmentation fault (core dumped)



//}


TEST(linear_dynamics, evolution_with_zero_initial_position)
{
    double jerk = 1.0;
    LinearDynamics linear_dynamics(jerk, 0.0, 0.0, 0.0);

    for(size_t t = 0; t < 100; t++)
    {
        EXPECT_TRUE(approx_equal(linear_dynamics.get_acceleration(t),
                                 t * jerk));
        EXPECT_TRUE(approx_equal(linear_dynamics.get_velocity(t),
                                 0.5 * t * t * jerk));
        EXPECT_TRUE(approx_equal(linear_dynamics.get_position(t),
                                 0.5 / 3.0 * t * t * t * jerk));
    }
}

// should be linear in initial state and control
TEST(linear_dynamics, check_linearity)
{
    Eigen::Vector4d parameters_a = Eigen::Vector4d(3.4, -5.6, 2.3, 7.2);
    Eigen::Vector4d parameters_b = Eigen::Vector4d(-2.1, -3.0, 5.5, -10.2);

    LinearDynamics linear_dynamics_a(parameters_a);
    LinearDynamics linear_dynamics_b(parameters_b);
    LinearDynamics linear_dynamics_sum(parameters_a + parameters_b);

    for(size_t t = 0; t < 100; t++)
    {
        EXPECT_TRUE(approx_equal(linear_dynamics_a.get_acceleration(t) +
                                 linear_dynamics_b.get_acceleration(t),
                                 linear_dynamics_sum.get_acceleration(t)));
        EXPECT_TRUE(approx_equal(linear_dynamics_a.get_velocity(t) +
                                 linear_dynamics_b.get_velocity(t),
                                 linear_dynamics_sum.get_velocity(t)));
        EXPECT_TRUE(approx_equal(linear_dynamics_a.get_position(t) +
                                 linear_dynamics_b.get_position(t),
                                 linear_dynamics_sum.get_position(t)));
    }
}


TEST(linear_dynamics, test_find_t_given_velocity_basic)
{
    Eigen::Vector4d parameters_basic = Eigen::Vector4d(1.0, -2.0, 0.0, 0.0);
    LinearDynamics dynamics_basic(parameters_basic);

    LinearDynamics::Vector solutions =
            dynamics_basic.find_t_given_velocity(- 3.0 / 2.0);

    EXPECT_TRUE(solutions.size() == 2);
    EXPECT_TRUE(contains(solutions, 1));
    EXPECT_TRUE(contains(solutions, 3));
    EXPECT_FALSE(contains(solutions, 4));
}


TEST(linear_dynamics, test_find_t_given_velocity_consistency)
{
    Eigen::Vector4d parameters_a = Eigen::Vector4d(3.4, -5.6, 2.3, 7.2);
    Eigen::Vector4d parameters_b = Eigen::Vector4d(-2.1, -3.0, 5.5, -10.2);
    LinearDynamics linear_dynamics_a(parameters_a);
    LinearDynamics linear_dynamics_b(parameters_b);

    for(size_t t = 0; t < 100; t++)
    {
        auto solutions_a = linear_dynamics_a.find_t_given_velocity(
                    linear_dynamics_a.get_velocity(t));
        EXPECT_TRUE(contains(solutions_a, t));
        auto solutions_b = linear_dynamics_b.find_t_given_velocity(
                    linear_dynamics_b.get_velocity(t));
        EXPECT_TRUE(contains(solutions_b, t));
    }
}



TEST(linear_dynamics_with_acceleration_constraint, test_time_evolution)
{
    Eigen::Matrix<double, 5, 1> parameters;
    parameters << -2.0, 15.0, 5.5, -10.2, 17.0;
    double jerk_duration = (17.0 + 15.0) / 2.0;


    LinearDynamicsWithAccelerationConstraint constrained_dynamics(parameters);
    LinearDynamics dynamics(parameters.topRows(4));
    LinearDynamicsWithAccelerationConstraint
            constrained_dynamics_b(-2.0,
                                   dynamics.get_acceleration(jerk_duration),
                                   dynamics.get_velocity(jerk_duration),
                                   dynamics.get_position(jerk_duration), 17.0);

    EXPECT_TRUE(approx_equal(dynamics.get_acceleration(jerk_duration), -17));
    EXPECT_TRUE(approx_equal(-17, constrained_dynamics.get_acceleration(
                                 jerk_duration + 100)));
    EXPECT_TRUE(approx_equal(dynamics.get_position(jerk_duration),
                             constrained_dynamics_b.get_position(0)));


    // make sure that ther is no discontinuity at jerk_duration ----------------
    double large_epsilon = std::sqrt(epsilon) / 100.0;
    double numeric, exact;

    numeric = constrained_dynamics.get_position(jerk_duration)
            + large_epsilon * constrained_dynamics.get_velocity(jerk_duration);
    exact = constrained_dynamics.get_position(jerk_duration + large_epsilon);
    ASSERT_TRUE(approx_equal(numeric, exact));

    numeric = constrained_dynamics.get_position(jerk_duration)
            - large_epsilon * constrained_dynamics.get_velocity(jerk_duration);
    exact = constrained_dynamics.get_position(jerk_duration - large_epsilon);
    ASSERT_TRUE(approx_equal(numeric, exact));

    // make sure dynamics coincide
    for(size_t t = 0; t < 10000; t++)
    {
        double acceleration_b = dynamics.get_acceleration(jerk_duration);
        double velocity_b =
                dynamics.get_velocity(jerk_duration) +
                dynamics.get_acceleration(jerk_duration) * t;
        double position_b =
                dynamics.get_position(jerk_duration) +
                dynamics.get_velocity(jerk_duration) * t +
                dynamics.get_acceleration(jerk_duration) * 0.5 * pow(t, 2);
        EXPECT_TRUE(approx_equal(acceleration_b,
                                 constrained_dynamics_b.get_acceleration(t)));
        EXPECT_TRUE(approx_equal(velocity_b,
                                 constrained_dynamics_b.get_velocity(t)));
        EXPECT_TRUE(approx_equal(position_b,
                                 constrained_dynamics_b.get_position(t)));



        EXPECT_TRUE(contains(constrained_dynamics.find_t_given_velocity(
                                 constrained_dynamics.get_velocity(t)), t));


        if(t <= jerk_duration)
        {
            EXPECT_TRUE(approx_equal(dynamics.get_acceleration(t),
                                     constrained_dynamics.get_acceleration(t)));
            EXPECT_TRUE(approx_equal(dynamics.get_velocity(t),
                                     constrained_dynamics.get_velocity(t)));
            EXPECT_TRUE(approx_equal(dynamics.get_position(t),
                                     constrained_dynamics.get_position(t)));
        }
        else
        {
            EXPECT_TRUE(approx_equal(constrained_dynamics_b.get_acceleration(t - jerk_duration),
                                     constrained_dynamics.get_acceleration(t)));
            EXPECT_TRUE(approx_equal(constrained_dynamics_b.get_velocity(t - jerk_duration),
                                     constrained_dynamics.get_velocity(t)));
            EXPECT_TRUE(approx_equal(constrained_dynamics_b.get_position(t - jerk_duration),
                                     constrained_dynamics.get_position(t)));
        }
    }
}

TEST(linear_dynamics_with_acceleration_constraint, consistency_with_linear_dynamics)
{

    LinearDynamicsWithAccelerationConstraint
            constrained_dynamics(-1.5, -13.0, 5.5, 10.2, 13.0);
    LinearDynamics  dynamics(0.0, -13.0, 5.5, 10.2);

    // make sure dynamics coincide
    for(size_t t = 0; t < 100; t++)
    {

        EXPECT_TRUE(approx_equal(dynamics.get_acceleration(t),
                                 constrained_dynamics.get_acceleration(t)));
        EXPECT_TRUE(approx_equal(dynamics.get_velocity(t),
                                 constrained_dynamics.get_velocity(t)));
        EXPECT_TRUE(approx_equal(dynamics.get_position(t),
                                 constrained_dynamics.get_position(t)));

        LinearDynamics::Vector solutions =
                dynamics.find_t_given_velocity(t - 50.0);
        LinearDynamics::Vector constrained_solutions =
                constrained_dynamics.find_t_given_velocity(t - 50.0);

        EXPECT_TRUE(solutions.size() == constrained_solutions.size());

        for(size_t i = 0; i < solutions.size(); i++)
        {
            EXPECT_TRUE(contains(solutions, constrained_solutions[i]));
            EXPECT_TRUE(contains(constrained_solutions, solutions[i]));
            EXPECT_TRUE(approx_equal(solutions[i], - (t - 50.0 -5.5) / 13.0));
        }
    }
}

TEST(linear_dynamics_with_acceleration_constraint,
     test_find_t_given_velocity_basic_1)
{
    LinearDynamicsWithAccelerationConstraint
            dynamics_basic(1.0, -2.0, 0.0, 0.0, 100.0);

    LinearDynamics::Vector solutions =
            dynamics_basic.find_t_given_velocity(- 3.0 / 2.0);

    EXPECT_TRUE(solutions.size() == 2);
    EXPECT_TRUE(contains(solutions, 1));
    EXPECT_TRUE(contains(solutions, 3));
    EXPECT_FALSE(contains(solutions, 4));
}


TEST(linear_dynamics_with_acceleration_constraint,
     test_find_t_given_velocity_basic_2)
{
    LinearDynamicsWithAccelerationConstraint
            dynamics_basic(1.0, -5.0, 3.0, -10.0, 10.0);

    LinearDynamics::Vector solutions =
            dynamics_basic.find_t_given_velocity(-9);

    EXPECT_TRUE(solutions.size() == 2);
    EXPECT_TRUE(contains(solutions, 4));
    EXPECT_TRUE(contains(solutions, 6));
    EXPECT_FALSE(contains(solutions, 7));
}


TEST(linear_dynamics_with_acceleration_constraint,
     test_find_t_given_velocity_basic_3)
{
    LinearDynamicsWithAccelerationConstraint
            dynamics(1.0, -1.0, 3.0, -2.0, 2.0);

    EXPECT_TRUE(approx_equal(dynamics.get_velocity(10.0), 18.5));

    LinearDynamics::Vector solutions = dynamics.find_t_given_velocity(18.5);

    EXPECT_TRUE(solutions.size() == 1);
    EXPECT_TRUE(approx_equal(solutions[0], 10.0));
}


TEST(linear_dynamics_with_acceleration_constraint,
     test_find_t_given_velocity_consistency)
{
    LinearDynamicsWithAccelerationConstraint
            dynamics_a(3.4, -5.6, 2.3, 7.2, 20);
    LinearDynamicsWithAccelerationConstraint
            dynamics_b(-2.1, -3.0, 5.5, -10.2, 44.3);

    for(size_t t = 0; t < 100; t++)
    {
        auto solutions_a = dynamics_a.find_t_given_velocity(
                    dynamics_a.get_velocity(t));
        EXPECT_TRUE(contains(solutions_a, t));
        auto solutions_b = dynamics_b.find_t_given_velocity(
                    dynamics_b.get_velocity(t));
        EXPECT_TRUE(contains(solutions_b, t));
    }
}


TEST(linear_dynamics_with_acceleration_constraint,
     will_exceed_jointly)
{
    LinearDynamicsWithAccelerationConstraint
            dynamics(-0.4, 2.6, 200.3, 7.2, 3);
    double max_t = dynamics.find_t_given_velocity(0).maxCoeff() * 2;

    std::vector<Eigen::Vector2d> trajectory;
    size_t n_iterations = 1000;
    for(size_t i = 0; i < n_iterations; i++)
    {
        double t = double(i) / n_iterations * max_t;

        Eigen::Vector2d point;
        point[0] = dynamics.get_velocity(t);
        point[1] = dynamics.get_position(t);
        trajectory.push_back(point);
    }

    for(auto& point : trajectory)
    {
        std::vector<Eigen::Vector2d> constraints;
        constraints.push_back(point + Eigen::Vector2d(0.001, 0.001));
        constraints.push_back(point - Eigen::Vector2d(epsilon, epsilon));

        constraints.push_back(point + Eigen::Vector2d(-epsilon, std::numeric_limits<double>::infinity()));
        constraints.push_back(point + Eigen::Vector2d(-epsilon, -std::numeric_limits<double>::infinity()));
        constraints.push_back(point + Eigen::Vector2d(0.001, -std::numeric_limits<double>::infinity()));

        constraints.push_back(point + Eigen::Vector2d(std::numeric_limits<double>::infinity(), -epsilon));
        constraints.push_back(point + Eigen::Vector2d(-std::numeric_limits<double>::infinity(), -epsilon));
        constraints.push_back(point + Eigen::Vector2d(-std::numeric_limits<double>::infinity(), 0.001));

        for(auto& constraint : constraints)
        {

            bool label = false;
            for(auto& point : trajectory)
            {
                if(point[0] > constraint[0] && point[1] > constraint[1])
                {
                    //                    std::cout << "point: " << point.transpose() << std::endl;
                    label = true;
                    break;
                }
            }

            double certificate_time;
            bool assigned_label =
                    dynamics.will_exceed_jointly(constraint[0], constraint[1],
                    certificate_time);

            if(label != assigned_label)
            {
                std::cout << "---------------------" << std::endl;
                std::cout << "constraint: " << constraint.transpose() << std::endl;

                std::cout << "label: " << label << "  assigned_label: " << assigned_label << std::endl;
                std::cout << "certificate time: " << certificate_time
                          << " certificate point: "
                          << dynamics.get_velocity(certificate_time) << ", "
                          << dynamics.get_position(certificate_time) << std::endl;
            }
            EXPECT_TRUE(label == assigned_label);
        }
    }
}



TEST(linear_dynamics_with_acceleration_constraint,
     will_exceed_jointly_2)
{
    LinearDynamicsWithAccelerationConstraint
            dynamics(-0.4, 20.6, -2.3, 7.2, 30);
    double max_t = dynamics.find_t_given_velocity(0).maxCoeff() * 2;

    std::vector<Eigen::Vector2d> trajectory;
    size_t n_iterations = 1000;
    for(size_t i = 0; i < n_iterations; i++)
    {
        double t = double(i) / n_iterations * max_t;

        Eigen::Vector2d point;
        point[0] = dynamics.get_velocity(t);
        point[1] = dynamics.get_position(t);
        trajectory.push_back(point);
    }

    for(auto& point : trajectory)
    {
        std::vector<Eigen::Vector2d> constraints;
        constraints.push_back(point + Eigen::Vector2d(0.001, 0.001));
        constraints.push_back(point - Eigen::Vector2d(epsilon, epsilon));

        constraints.push_back(point + Eigen::Vector2d(-epsilon, std::numeric_limits<double>::infinity()));
        constraints.push_back(point + Eigen::Vector2d(-epsilon, -std::numeric_limits<double>::infinity()));
        constraints.push_back(point + Eigen::Vector2d(0.001, -std::numeric_limits<double>::infinity()));

        constraints.push_back(point + Eigen::Vector2d(std::numeric_limits<double>::infinity(), -epsilon));
        constraints.push_back(point + Eigen::Vector2d(-std::numeric_limits<double>::infinity(), -epsilon));
        constraints.push_back(point + Eigen::Vector2d(-std::numeric_limits<double>::infinity(), 0.001));

        for(auto& constraint : constraints)
        {

            bool label = false;
            for(auto& point : trajectory)
            {
                if(point[0] > constraint[0] && point[1] > constraint[1])
                {
                    //                    std::cout << "point: " << point.transpose() << std::endl;
                    label = true;
                    break;
                }
            }

            double certificate_time;
            bool assigned_label =
                    dynamics.will_exceed_jointly(constraint[0], constraint[1],
                    certificate_time);

            if(label != assigned_label)
            {
                std::cout << "---------------------" << std::endl;
                std::cout << "constraint: " << constraint.transpose() << std::endl;

                std::cout << "label: " << label << "  assigned_label: " << assigned_label << std::endl;
                std::cout << "certificate time: " << certificate_time
                          << " certificate point: "
                          << dynamics.get_velocity(certificate_time) << ", "
                          << dynamics.get_position(certificate_time) << std::endl;
            }
            EXPECT_TRUE(label == assigned_label);
        }
    }
}


TEST(linear_dynamics_with_acceleration_constraint,
     will_deceed_jointly)
{
    LinearDynamicsWithAccelerationConstraint
            dynamics(0.4, 2.6, -200.3, 7.2, 3);
    double max_t = dynamics.find_t_given_velocity(0).maxCoeff() * 2;

    std::vector<Eigen::Vector2d> trajectory;
    size_t n_iterations = 1000;
    for(size_t i = 0; i < n_iterations; i++)
    {
        double t = double(i) / n_iterations * max_t;

        Eigen::Vector2d point;
        point[0] = dynamics.get_velocity(t);
        point[1] = dynamics.get_position(t);
        trajectory.push_back(point);
    }

    for(auto& point : trajectory)
    {
        std::vector<Eigen::Vector2d> constraints;
        constraints.push_back(point - Eigen::Vector2d(0.001, 0.001));
        constraints.push_back(point + Eigen::Vector2d(epsilon, epsilon));

        constraints.push_back(point + Eigen::Vector2d(epsilon, std::numeric_limits<double>::infinity()));
        constraints.push_back(point + Eigen::Vector2d(epsilon, -std::numeric_limits<double>::infinity()));
        constraints.push_back(point + Eigen::Vector2d(-0.001, std::numeric_limits<double>::infinity()));

        constraints.push_back(point + Eigen::Vector2d(std::numeric_limits<double>::infinity(), epsilon));
        constraints.push_back(point + Eigen::Vector2d(-std::numeric_limits<double>::infinity(), epsilon));
        constraints.push_back(point + Eigen::Vector2d(std::numeric_limits<double>::infinity(), -0.001));

        for(auto& constraint : constraints)
        {

            bool label = false;
            for(auto& point : trajectory)
            {
                if(point[0] < constraint[0] && point[1] < constraint[1])
                {
                    //                    std::cout << "point: " << point.transpose() << std::endl;
                    label = true;
                    break;
                }
            }

            double certificate_time;
            bool assigned_label =
                    dynamics.will_deceed_jointly(constraint[0], constraint[1],
                    certificate_time);

            if(label != assigned_label)
            {
                std::cout << "---------------------" << std::endl;
                std::cout << "constraint: " << constraint.transpose() << std::endl;

                std::cout << "label: " << label << "  assigned_label: " << assigned_label << std::endl;
                std::cout << "certificate time: " << certificate_time
                          << " certificate point: "
                          << dynamics.get_velocity(certificate_time) << ", "
                          << dynamics.get_position(certificate_time) << std::endl;
            }
            EXPECT_TRUE(label == assigned_label);
        }
    }
}



TEST(find_max_admissible_acceleration, generated_trajectories)
{
    srand(0);

    for(size_t unused = 0; unused < 100; unused++)
    {
        // initialize parameters randomly --------------------------------------


        double initial_velocity = sample_uniformely(-20.0, 20.0);
        double initial_position = sample_uniformely(-20.0, 20.0);
        NonnegDouble abs_jerk_limit = sample_uniformely(epsilon, 20.0);
        NonnegDouble abs_acceleration_limit = sample_uniformely(epsilon, 20.0);

        double initial_acceleration = sample_uniformely(-abs_acceleration_limit,
                                                        abs_acceleration_limit);



        // find some constraints which is just barely satisfied --------------------
        LinearDynamicsWithAccelerationConstraint dynamics(-abs_jerk_limit,
                                                          initial_acceleration,
                                                          initial_velocity,
                                                          initial_position,
                                                          abs_acceleration_limit);

        double max_t = 20.0;
        if(dynamics.find_t_given_velocity(0).size() > 0)
        {
            max_t = dynamics.find_t_given_velocity(0).maxCoeff() * 2;
        }

        size_t n_iterations = 10000;
        int T = rand() % n_iterations;
        Eigen::Vector2d constraint;
        constraint[0] = dynamics.get_velocity(T);
        constraint[1] = dynamics.get_position(T);

        Eigen::Vector2d position_constraint(-std::numeric_limits<double>::infinity(),
                                            -std::numeric_limits<double>::infinity());
        Eigen::Vector2d velocity_constraint(-std::numeric_limits<double>::infinity(),
                                            -std::numeric_limits<double>::infinity());

        for(size_t i = 0; i < n_iterations; i++)
        {
            double t = double(i) / n_iterations * max_t;
            Eigen::Vector2d point;
            point[0] = dynamics.get_velocity(t);
            point[1] = dynamics.get_position(t);
            if(point[0] > constraint[0] && point[1] > constraint[1])
                constraint = point;
            velocity_constraint[0] = std::max(velocity_constraint[0], point[0]);
            position_constraint[1] = std::max(position_constraint[1], point[1]);
        }

        // test that we get the same initial acceleration --------------------------
        for(auto& c : {constraint, position_constraint, velocity_constraint})
        {


            double max_admissible_initial_acceleration =
                    find_max_admissible_acceleration(
                        initial_velocity,
                        initial_position,
                        c[0],
                    c[1],
                    abs_jerk_limit,
                    abs_acceleration_limit);

            if(std::fabs(initial_acceleration -
                         max_admissible_initial_acceleration) > 0.02)
            {
                if(max_admissible_initial_acceleration - 0.0001 >
                        -abs_acceleration_limit)
                {
                    LinearDynamicsWithAccelerationConstraint
                            below_limit_dynamics(-abs_jerk_limit,
                                                 max_admissible_initial_acceleration
                                                 - 0.0001,
                                                 initial_velocity,
                                                 initial_position,
                                                 abs_acceleration_limit);
                    ASSERT_TRUE(below_limit_dynamics.will_exceed_jointly(c[0], c[1])
                            == false);
                }


                if(max_admissible_initial_acceleration + 0.0001 <
                        abs_acceleration_limit)
                {
                    LinearDynamicsWithAccelerationConstraint
                            above_limit_dynamics(-abs_jerk_limit,
                                                 max_admissible_initial_acceleration
                                                 + 0.0001,
                                                 initial_velocity,
                                                 initial_position,
                                                 abs_acceleration_limit);
                    ASSERT_TRUE(above_limit_dynamics.will_exceed_jointly(c[0], c[1])
                            == true);
                }


                //                std::cout << "------------------------------------  " << std::endl;


                //                dynamics.print_parameters();

                //                std::cout << "constraint " << c << std::endl;

                //                std::cout << " initial_acceleration " << initial_acceleration
                //                          << " max_admissible_initial_acceleration "
                //                          << max_admissible_initial_acceleration << std::endl;

                //                bool will_exceed_epsilon = dynamics.will_exceed_jointly(c[0], c[1]);

                //                bool will_exceed_minus_epsilon = dynamics.will_exceed_jointly(c[0] - epsilon, c[1] - epsilon);


                //                bool limit_dynamics_will_exceed = limit_dynamics.will_exceed_jointly(c[0] - epsilon, c[1] - epsilon);



                //                std::cout << " will_exceed_epsilon " << will_exceed_epsilon
                //                          << " will_exceed_minus_epsilon " << will_exceed_minus_epsilon
                //                          << " limit_dynamics_will_exceed " << limit_dynamics_will_exceed

                //                          << std::endl;

                ASSERT_TRUE(approx_equal(c[0], initial_velocity) ||
                        approx_equal(c[1], initial_position));
            }

            //            ASSERT_TRUE(std::fabs(initial_acceleration -
            //                                  max_admissible_initial_acceleration) <= 0.001);
        }
    }
}


TEST(find_max_admissible_acceleration, random_points)
{
    srand(0);

    for(size_t unused = 0; unused < 10000; unused++)
    {
        // initialize parameters randomly --------------------------------------
        double initial_velocity = sample_uniformely(-20.0, 20.0);
        double initial_position = sample_uniformely(-20.0, 20.0);

        double max_velocity = sample_uniformely(-20.0, 20.0);
        double max_position = sample_uniformely(-20.0, 20.0);

        NonnegDouble abs_jerk_limit = sample_uniformely(epsilon, 20.0);
        NonnegDouble abs_acceleration_limit = sample_uniformely(epsilon, 20.0);


        double max_admissible_acceleration =
                find_max_admissible_acceleration(
                    initial_velocity,
                    initial_position,
                    max_velocity,
                    max_position,
                    abs_jerk_limit,
                    abs_acceleration_limit);

        if(std::fabs(max_admissible_acceleration - 0.0001) <=
                abs_acceleration_limit)
        {
            LinearDynamicsWithAccelerationConstraint
                    below_limit_dynamics(-abs_jerk_limit,
                                         max_admissible_acceleration
                                         - 0.0001,
                                         initial_velocity,
                                         initial_position,
                                         abs_acceleration_limit);
            ASSERT_TRUE(below_limit_dynamics.will_exceed_jointly(max_velocity, max_position)
                        == false);
        }
        if(std::fabs(max_admissible_acceleration + 0.0001) <=
                abs_acceleration_limit)
        {
            LinearDynamicsWithAccelerationConstraint
                    above_limit_dynamics(-abs_jerk_limit,
                                         max_admissible_acceleration
                                         + 0.0001,
                                         initial_velocity,
                                         initial_position,
                                         abs_acceleration_limit);
            ASSERT_TRUE(above_limit_dynamics.will_exceed_jointly(max_velocity, max_position)
                        == true);
        }
    }
}


TEST(find_min_admissible_acceleration, random_points)
{
    srand(0);

    for(size_t unused = 0; unused < 10; unused++)
    {
        // initialize parameters randomly --------------------------------------
        double initial_velocity = sample_uniformely(-20.0, 20.0);
        double initial_position = sample_uniformely(-20.0, 20.0);

        double min_velocity = sample_uniformely(-20.0, 20.0);
        double min_position = sample_uniformely(-20.0, 20.0);

        NonnegDouble abs_jerk_limit = sample_uniformely(epsilon, 20.0);
        NonnegDouble abs_acceleration_limit = sample_uniformely(epsilon, 20.0);


        double min_admissible_acceleration =
                find_min_admissible_acceleration(
                    initial_velocity,
                    initial_position,
                    min_velocity,
                    min_position,
                    abs_jerk_limit,
                    abs_acceleration_limit);

        if(std::fabs(min_admissible_acceleration - 0.0001) <=
                abs_acceleration_limit)
        {
            LinearDynamicsWithAccelerationConstraint
                    below_limit_dynamics(abs_jerk_limit,
                                         min_admissible_acceleration
                                         - 0.0001,
                                         initial_velocity,
                                         initial_position,
                                         abs_acceleration_limit);
            ASSERT_TRUE(below_limit_dynamics.will_deceed_jointly(min_velocity,
                                                                 min_position)
                        == true);
        }
        if(std::fabs(min_admissible_acceleration + 0.0001) <=
                abs_acceleration_limit)
        {
            LinearDynamicsWithAccelerationConstraint
                    above_limit_dynamics(abs_jerk_limit,
                                         min_admissible_acceleration
                                         + 0.0001,
                                         initial_velocity,
                                         initial_position,
                                         abs_acceleration_limit);
            ASSERT_TRUE(above_limit_dynamics.will_deceed_jointly(min_velocity,
                                                                 min_position)
                        == false);
        }
    }
}




