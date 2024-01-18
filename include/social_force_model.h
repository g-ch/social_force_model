#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

class SocialForceModel {
public:
    SocialForceModel(): force_factor_desired(1.0), force_factor_social(1.0), force_factor_obstacle(1.0)
    {};
    ~SocialForceModel(){};

    /// @brief Sets the force factor for the desired force
    /// @param force_factor The force factor for the desired force
    void setForceFactorDesired(double force_factor)
    {
        force_factor_desired = force_factor;
    }

    /// @brief Sets the force factor for the social force
    /// @param force_factor The force factor for the social force
    void setForceFactorSocial(double force_factor)
    {
        force_factor_social = force_factor;
    }

    /// @brief Sets the force factor for the obstacle force
    /// @param force_factor The force factor for the obstacle force
    void setForceFactorObstacle(double force_factor)
    {
        force_factor_obstacle = force_factor;
    }
    

    /// @brief Calculates the social force.
    /// @param robot_state The state of the robot.x,y,theta,vx,vy,radius
    /// @param static_obj_states The states of the static obstacles. x,y,radius
    /// @param others_states The states of the other agents. Each agen's state is a vector composed of x,y,theta,vx,vy,radius
    /// @param goal The goal of the robot
    /// @param pref_speed The preferred speed of the robot
    /// @return The social force
    Eigen::Vector2d sfmStep(const std::vector<double> &robot_state, const std::vector<std::vector<double>> &static_obj_states, const std::vector<std::vector<double>> &others_states, const Eigen::Vector2d &goal, Eigen::Vector2d &goal_force, Eigen::Vector2d &static_obstacle_force, Eigen::Vector2d &social_force, double pref_speed = 0.5)
    {
        // Check if robot_state has 6 elements
        if (robot_state.size() != 6)
        {
            std::cout << "robot_state has " << robot_state.size() << " elements, but should have 5 elements." << std::endl;
            return Eigen::Vector2d::Zero();
        }

        Eigen::Vector2d robot_position = Eigen::Vector2d::Zero();
        robot_position[0] = robot_state[0];
        robot_position[1] = robot_state[1];

        Eigen::Vector2d robot_velocity = Eigen::Vector2d::Zero();
        robot_velocity[0] = robot_state[3];
        robot_velocity[1] = robot_state[4];

        double robot_radius = robot_state[5];

        /****** Goal force ******/
        goal_force = goalForce(robot_position, robot_velocity, goal, pref_speed);
        goal_force *= force_factor_desired;
        std::cout << "goal_force: " << goal_force.transpose() << std::endl;

        /***** Static obstacle force ******/
        static_obstacle_force = Eigen::Vector2d::Zero();
        for(int i = 0; i < static_obj_states.size(); i++)
        {
            // Check if static_obj_states[i] has 3 elements
            if (static_obj_states[i].size() != 3)
            {
                std::cout << "static_obj_states[" << i << "] has " << static_obj_states[i].size() << " elements, but should have 3 elements." << std::endl;
                return Eigen::Vector2d::Zero();
            }

            Eigen::Vector2d obstacle_position = Eigen::Vector2d::Zero();
            obstacle_position[0] = static_obj_states[i][0];
            obstacle_position[1] = static_obj_states[i][1];

            double obstacle_radius = static_obj_states[i][2];
            Eigen::Vector2d cloest_position = findCloestPointOnCircle(obstacle_position, obstacle_radius, robot_position);

            Eigen::Vector2d obstacle_force = repulsiveObstForce(robot_position, robot_velocity, robot_radius, cloest_position, force_factor_obstacle);

            static_obstacle_force += obstacle_force;
        }

        std::cout << "static_obstacle_force: " << static_obstacle_force.transpose() << std::endl;

        /****** Social force ******/
        social_force = Eigen::Vector2d::Zero();
        for(int i = 0; i < others_states.size(); i++)
        {
            // Check if others_states[i] has 5 elements
            if (others_states[i].size() != 6)
            {
                std::cout << "others_states[" << i << "] has " << others_states[i].size() << " elements, but should have 5 elements." << std::endl;
                return Eigen::Vector2d::Zero();
            }

            Eigen::Vector2d other_position = Eigen::Vector2d::Zero();
            other_position[0] = others_states[i][0];
            other_position[1] = others_states[i][1];

            Eigen::Vector2d other_velocity = Eigen::Vector2d::Zero();
            other_velocity[0] = others_states[i][3];
            other_velocity[1] = others_states[i][4];

            Eigen::Vector2d interactionVector = Eigen::Vector2d::Zero();
            double radius_other = others_states[i][5];

            Eigen::Vector2d ped_force = repulsivePedForce(robot_position, robot_velocity, robot_radius, radius_other, goal, other_position, other_velocity, force_factor_social, interactionVector);

            social_force += ped_force;
        }

        std::cout << "social_force: " << social_force.transpose() << std::endl;

        Eigen::Vector2d total_force = goal_force + static_obstacle_force + social_force;
        std::cout << "total_force: " << total_force.transpose() << std::endl;

        return total_force;
    }



private:
    /// @brief Computes the goal force
    /// @param position The position of the robot
    /// @param current_vel The velocity of the robot
    /// @param goal The goal of the robot
    /// @param pref_speed The preferred speed of the robot
    /// @param rel_time The relaxation time of the robot
    /// @return The goal force
    inline Eigen::Vector2d goalForce(const Eigen::Vector2d &position, const Eigen::Vector2d &current_vel, const Eigen::Vector2d &goal, double pref_speed, double rel_time=0.54)
    {
        Eigen::Vector2d diff = goal - position;
        double dist = std::sqrt(diff[0]*diff[0] + diff[1]*diff[1]);
        Eigen::Vector2d desiredDirection = diff / dist;
        Eigen::Vector2d force = (desiredDirection * pref_speed - current_vel) / rel_time;
        return force;
    }

    /// @brief Computes the repulsive force from a static obstalce
    /// @param position The position of the robot
    /// @param velocity The velocity of the robot
    /// @param radius The radius of the robot
    /// @param obstacle_position The position of the obstacle
    /// @return The repulsive force from the obstacle
    inline Eigen::Vector2d repulsiveObstForce(const Eigen::Vector2d &position, const Eigen::Vector2d &velocity, double radius, const Eigen::Vector2d &obstacle_position, double forceObstacle = 0.8)
    {
        Eigen::Vector2d diff = position - obstacle_position;
        double dist = std::sqrt(diff[0]*diff[0] + diff[1]*diff[1]);
        Eigen::Vector2d diff_normalized = diff / dist;

        double dist_without_r = dist - radius;
        double forceAmount = -dist_without_r/forceObstacle; //todo check what if dist_without-r is negative

        Eigen::Vector2d force = forceAmount * diff_normalized;

        return force;
    }

    /// @brief Computes the repulsive force from a dynamic obstacle
    /// @param position The position of the robot
    /// @param velocity The velocity of the robot
    /// @param radius The radius of the robot
    /// @param obstacle_position The position of the obstacle
    /// @param obstacle_velocity The velocity of the obstacle
    /// @return The repulsive force from the obstacle
    inline Eigen::Vector2d repulsivePedForce(const Eigen::Vector2d &position, const Eigen::Vector2d &velocity, double r_ego, double r_other, const Eigen::Vector2d &goal, const Eigen::Vector2d &other_ped_pos, const Eigen::Vector2d &other_ped_vel, double forceFactorSocial, Eigen::Vector2d &interactionVector)
    {
        double A = 4.5;
        double lambdaImportance = 1.5;
        double gamma = 0.35;
        double n = 2;
        double n_prime = 3;

        Eigen::Vector2d diff = other_ped_pos - position;
        double dist = std::sqrt(diff[0]*diff[0] + diff[1]*diff[1]);
        Eigen::Vector2d diffDirection = diff / dist;

        double combined_radius = r_ego + r_other;
        double dist_without_r = dist - combined_radius;
        Eigen::Vector2d diffwithoutr = diff - combined_radius * diffDirection;
        Eigen::Vector2d diffwithoutrDirection = diffwithoutr/dist_without_r;

        Eigen::Vector2d velDiff = velocity-other_ped_vel;
        double velDiff_norm = std::sqrt(velDiff[0]*velDiff[0] + velDiff[1]*velDiff[1]);
        Eigen::Vector2d velDiffDirection = velDiff/velDiff_norm;

        interactionVector = lambdaImportance * velDiffDirection + diffDirection;
        double interactionLength = std::sqrt(interactionVector[0]*interactionVector[0] + interactionVector[1]*interactionVector[1]);
        Eigen::Vector2d interactionDirection = interactionVector / interactionLength;

        Eigen::Vector2d v = interactionDirection;
        Eigen::Vector2d w = diffwithoutrDirection;

        double inner = v.dot(w);
        double norms = interactionDirection.norm() * diffwithoutrDirection.norm();

        double cos = inner / norms;
        if (cos >= 1.0)
            cos = 0.999;
        if (cos <= -1.0)
            cos = -0.999;
        double theta = std::acos(cos);

        double B = gamma * interactionLength;
        double eps = 0.05;
        double thetaRad = theta + B * eps; //* ca.sign(forceFactorSocial)  # math.radians(theta[0])

        double forceVelocityAmount = - A * std::exp(-dist_without_r / B - (n_prime * B * thetaRad) * (n_prime * B * thetaRad));
        double forceAngleAmount = - A * std::copysign(theta, 1) * std::exp(-dist_without_r / B - (n * B * thetaRad) * (n * B * thetaRad));

        Eigen::Vector2d forceVelocity = forceVelocityAmount * interactionDirection;
        Eigen::Vector2d interactionDirectionPerpendicular = Eigen::Vector2d::Zero();
        interactionDirectionPerpendicular[0] = -interactionDirection[1];
        interactionDirectionPerpendicular[1] = interactionDirection[0];

        Eigen::Vector2d forceAngle = forceAngleAmount * interactionDirectionPerpendicular;


        return forceVelocity + forceAngle;
    }


    /// @brief Computes the cloest point on a circle to a given point
    /// @param circle_center The center of the circle
    /// @param radius The radius of the circle
    /// @param robot_position The position of the robot
    Eigen::Vector2d findCloestPointOnCircle(Eigen::Vector2d &circle_center, double radius, Eigen::Vector2d &robot_position)
    {
        Eigen::Vector2d diff = robot_position - circle_center;
        double dist = std::sqrt(diff[0]*diff[0] + diff[1]*diff[1]);
        Eigen::Vector2d diff_normalized = diff / dist;

        Eigen::Vector2d cloest_point = circle_center + diff_normalized * radius;

        return cloest_point;
    }

private:
    double force_factor_desired;
    double force_factor_social;
    double force_factor_obstacle;

};