#ifndef DIFFDRIVE_ARDUINO_WHEEL_HPP
#define DIFFDRIVE_ARDUINO_WHEEL_HPP

#include <string>
#include <cmath>
#include <iostream> // Include the necessary header for printing

class BaseWheel
{
public:
    std::string name = "";
    int enc = 0;
    double cmd = 0;
    double pos = 0;
    double vel = 0;
    double rads_per_count = 0;

    BaseWheel() = default;

    BaseWheel(const std::string &wheel_name, int counts_per_rev)
    {
        setup(wheel_name, counts_per_rev);
    }

    void setup(const std::string &wheel_name, int counts_per_rev)
    {
        name = wheel_name;
        rads_per_count = (2 * M_PI) / counts_per_rev;
    }

    double calc_enc_angle()
    {
        return enc * rads_per_count;
    }
};

class ForwardBackwardWheel : public BaseWheel
{
public:
    using BaseWheel::BaseWheel; // Inherit constructors from the base class

    void moveForward(double speed)
    {
        // Add code to move the wheel forward at the given speed
        // For this example, we'll just print the action.
        std::cout << "Moving " << name << " forward at speed: " << speed << std::endl;
    }

    void moveBackward(double speed)
    {
        // Add code to move the wheel backward at the given speed
        // For this example, we'll just print the action.
        std::cout << "Moving " << name << " backward at speed: " << speed << std::endl;
    }
};

class TwistWheel : public BaseWheel
{
public:
    using BaseWheel::BaseWheel; // Inherit constructors from the base class

    void twist(double angular_speed)
    {
        // Add code to make the wheel perform twists at the given angular speed
        // For this example, we'll just print the action.
        std::cout << "Twisting " << name << " at angular speed: " << angular_speed << std::endl;
    }
};

#endif // DIFFDRIVE_ARDUINO_WHEEL_HPP
