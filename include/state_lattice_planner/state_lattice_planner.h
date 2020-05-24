/**
* @file state_lattice_planner.h
* @brief C++ implementation for State Lattice Planner
* @author AMSL
*/
#ifndef __STATE_LATTICE_PLANNER_H
#define __STATE_LATTICE_PLANNER_H

#include <sstream>

#include <Eigen/Dense>
#include <omp.h>

#include <trajectory_generator/motion_model_diff_drive.h>
#include <trajectory_generator/trajectory_generator_diff_drive.h>
#include "state_lattice_planner/lookup_table_utils.h"
#include "state_lattice_planner/obstacle_map.h"

/**
 * @brief Class for state lattice planning
 */
class StateLatticePlanner
{
public:
    /**
     * @brief Class for parameters of trajectory sampling
     */
    class SamplingParams
    {
    public:
        SamplingParams(void);
        SamplingParams(const int, const int, const double, const double);
        SamplingParams(const int, const int, const double, const double, const double);

        int n_p;// num of sampling positions
        int n_h;// num of sampling angle by position
        double length;// sample trajectory length
        double max_alpha;// max trajectories angle
        double min_alpha;// min trajectories angle
        double span_alpha;// max - min alpha
        double max_psi;// max heading angle
        double min_psi;// min trajectories angle
        double span_psi;// max - min psi
    private:
    };

    /**
     * @brief Constructor
     */
    StateLatticePlanner(void);

    /**
     * @brief Set parameters for trajectory optimization
     * @param[in] max_iteration_ Maximum number of iteration for trajectory optimization
     * @param[in] tolerance_ If the cost of the optimization is less than this value, it is considered converged
     * */
    void set_optimization_params(int, double);
    /**
     * @brief Set parameters for trajectory terminal state sampling
     * @param[in] sampling_params_ Parameters for trajectory terminal state sampling
     */
    void set_sampling_params(const SamplingParams&);
    /**
     * @brief Set target velocity
     * @param[in] v Target velocity [m/s]
     */
    void set_target_velocity(double);
    /**
     * @brief Set motion parameters
     * @param[in] max_acceleration_ Maximum linear acceleration [m/ss]
     * @param[in] max_yawrate_ Maximum angular velocity [rad/s]
     * @param[in] max_d_yawrate_ Maximum angular acceleration [rad/ss]
     */
    void set_motion_params(double, double, double);
    /**
     * @brief Set vehicle parameters
     * @param[in] wheel_radius_ Wheel radius [m]
     * @param[in] tread_ Tread [m]
     */
    void set_vehicle_params(double, double);
    /**
     * @brief Load lookup table from csv
     * @param[in] LOOKUP_TABLE_FILE_NAME file path of the lookup table
     */
    void load_lookup_table(const std::string&);
    /**
     * @brief Generate biased polar states
     * @param[in] n_s Parameter for sampling
     * @param[in] goal Goal pose (x, y, yaw)
     * @param[in] target_velocity Target velocity [m/s]
     * @param[out] states Terminal states
     */
    void generate_biased_polar_states(const int, const Eigen::Vector3d&, double, std::vector<Eigen::Vector3d>&);
    /**
     * @brief Sample states from specified angle
     * @param[in] sample_angle Values between [0, 1] representing angles
     * @param[out] states Terminal states
     */
    void sample_states(const std::vector<double>&, std::vector<Eigen::Vector3d>&);
    /**
     * @brief Generate trajectories to specified terminal states
     * @param[in] boudary_states Terminal states
     * @param[in] velocity Initial velocity [m/s]
     * @param[in] angular_velocity Initial yawrate [rad/s]
     * @param[in] target_velocity Target velocity [m/s]
     * @param[out] trajectories Generated trajectories
     */
    bool generate_trajectories(const std::vector<Eigen::Vector3d>&, const double, const double, const double, std::vector<MotionModelDiffDrive::Trajectory>&);
    /**
     * @brief Pickup a trajectory to execute from candidates
     * @param[in] candidate_trajectories Candidate trajectories
     * @param[in] goal Goal pose (x, y, yaw)
     * @param[in] output The adopted trajectory
     * @return Whether the output trajectory is valid or not
     */
    bool pickup_trajectory(const std::vector<MotionModelDiffDrive::Trajectory>&, const Eigen::Vector3d&, MotionModelDiffDrive::Trajectory&);
    /**
     * @brief Not implemented
     */
    void load_lookup_table(void);
    /**
     * @brief Not implemented
     */
    void get_optimized_param_from_lookup_table(const Eigen::Vector3d, const double, const double, MotionModelDiffDrive::ControlParams&);
    /**
     * @brief Get target velocity from goal direction
     * @param[in] goal Goal pose (x, y, yaw)
     * @return Target velocity [m/s]
     */
    double get_target_velocity(const Eigen::Vector3d&);
    /**
     * @brief Generate bresenhams line from given trajectory
     * @param[in] trajectory Original trajectory
     * @param[out] Bresenhams line trajectory
     */
    void generate_bresemhams_line(const std::vector<Eigen::Vector3d>&, const double&, std::vector<Eigen::Vector3d>&);
    /**
     * @brief Check collision in the obstacle map
     * @param[in] map Obstacle map
     * @param[in] trajectory Trajectory
     * @return True if the given trajectory collides with an obstacle
     */
    bool check_collision(const state_lattice_planner::ObstacleMap<int>&, const std::vector<Eigen::Vector3d>&);
    /**
     * @brief Check collision in the obstacle map. Ignoring obstacles out of range
     * @param[in] map Obstacle map
     * @param[in] trajectory Trajectory
     * @param[in] range Range [m]
     * @return True if the given trajectory collides with an obstacle
     */
    bool check_collision(const state_lattice_planner::ObstacleMap<int>&, const std::vector<Eigen::Vector3d>&, double);

protected:
    double HZ;
    int MAX_ITERATION;
    double OPTIMIZATION_TOLERANCE;
    double TARGET_VELOCITY;
    double MAX_ACCELERATION;
    double MAX_YAWRATE;
    double MAX_D_YAWRATE;
    double MAX_WHEEL_ANGULAR_VELOCITY;
    double WHEEL_RADIUS;
    double TREAD;
    bool VERBOSE;
    bool ENABLE_CONTROL_SPACE_SAMPLING;

    SamplingParams sampling_params;
    LookupTableUtils::LookupTable lookup_table;
};

#endif //__STATE_LATTICE_PLANNER_H
