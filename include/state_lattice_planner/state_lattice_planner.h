#ifndef __STATE_LATTICE_PLANNER_H
#define __STATE_LATTICE_PLANNER_H

#include <sstream>

#include <Eigen/Dense>
#include <omp.h>

#include <trajectory_generator/motion_model_diff_drive.h>
#include <trajectory_generator/trajectory_generator_diff_drive.h>
#include "state_lattice_planner/lookup_table_utils.h"
#include "state_lattice_planner/obstacle_map.h"

class StateLatticePlanner
{
public:
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

    StateLatticePlanner(void);

    void set_optimization_params(int, double);
    void set_sampling_params(const SamplingParams&);
    void set_target_velocity(double);
    void set_motion_params(double, double, double);
    void set_vehicle_params(double, double);
    void load_lookup_table(const std::string&);
    void generate_biased_polar_states(const int, const Eigen::Vector3d&, double, std::vector<Eigen::Vector3d>&);
    void sample_states(const std::vector<double>&, std::vector<Eigen::Vector3d>&);
    bool generate_trajectories(const std::vector<Eigen::Vector3d>&, const double, const double, const double, std::vector<MotionModelDiffDrive::Trajectory>&);
    bool pickup_trajectory(const std::vector<MotionModelDiffDrive::Trajectory>&, const Eigen::Vector3d&, MotionModelDiffDrive::Trajectory&);
    void load_lookup_table(void);
    void get_optimized_param_from_lookup_table(const Eigen::Vector3d, const double, const double, MotionModelDiffDrive::ControlParams&);
    double get_target_velocity(const Eigen::Vector3d&);
    void generate_bresemhams_line(const std::vector<Eigen::Vector3d>&, const double&, std::vector<Eigen::Vector3d>&);
    bool check_collision(const state_lattice_planner::ObstacleMap<int>&, const std::vector<Eigen::Vector3d>&);
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
