#ifndef TURTLELIB_DIFF_DRIVE_INCLUDE_GUARD_HPP
#define TURTLELIB_DIFF_DRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Models the kinematics of a differential drive robot


#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

namespace turtlelib
{
    /// \brief represent a diff drive robot's wheels
    struct Wheels
    {
        /// \brief the left wheel's position (rad)
        double phi_l = 0.0;

        /// \brief the right wheel's position (rad)
        double phi_r = 0.0;
    };

    /// \brief represent a robot's configuration
    struct RobotConfig
    {
        /// \brief the robot's theta position (rad)
        double theta = 0.0;
        
        /// \brief the robot's x-coordinate
        double x = 0.0;

        /// \brief the robot's y-coordinate
        double y = 0.0;

    };
    
    
    class DiffDrive
    {
        private:

        double wheel_track;
        double wheel_radius;
        Wheels wheels;
        RobotConfig q;

        public:

        /// \brief Initialize DiffDrive constructor
        DiffDrive();

        /// \brief create a DiffDrive object
        /// \param wheel_track - full distance between wheels
        /// \param wheel_radius - radius of wheels
        /// \param wheels - Wheels object
        /// \param q - RobotConfig object
        DiffDrive(double wheel_track, double wheel_radius, Wheels wheels, RobotConfig q);

        /// \brief get wheel position
        /// \return Wheels object 
        Wheels get_wheels() const;

        /// \brief get robot configuration
        /// \brief RobotConfig object
        RobotConfig get_robot_config() const;

        /// \brief set new wheels position
        /// \param new_wheels - new wheels position
        void set_wheels(Wheels new_wheels);

        /// \brief set new robot configuration
        /// \param new_q - new robot configuration
        void set_robot_config(RobotConfig q_new);

        /// \brief Given new relative wheel positions, update the robot configuration
        /// \param delta_wheels change in wheel position
        /// \return updates the robot's configuration
        void forward_kinematic_update(Wheels delta_wheels);

        /// \brief Compute wheel velocities to move at given twist
        /// \param twist - Twist2D object
        /// \returns Wheels object
        Wheels inverse_kinematics(Twist2D twist);





    };
}

#endif