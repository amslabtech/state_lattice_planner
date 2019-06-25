#include "state_lattice_planner/lookup_table_generator.h"

LookupTableGenerator::LookupTableGenerator(void)
:local_nh("~")
{
    local_nh.param("MIN_X", MIN_X, {1.0});
    local_nh.param("MAX_X", MAX_X, {5.0});
    local_nh.param("DELTA_X", DELTA_X, {1.0});
    local_nh.param("MAX_Y", MAX_Y, {2.0});
    local_nh.param("DELTA_Y", DELTA_Y, {1.0});
    local_nh.param("MAX_YAW", MAX_YAW, {M_PI / 3.0});
    local_nh.param("DELTA_YAW", DELTA_YAW, {M_PI / 3.0});
    local_nh.param("LOOKUP_TABLE_FILE_NAME", LOOKUP_TABLE_FILE_NAME, {std::string(std::getenv("HOME")) + "/lookup_table.csv"});
    local_nh.param("MAX_ACCELERATION", MAX_ACCELERATION, {1.0});
    local_nh.param("TARGET_VELOCITY", TARGET_VELOCITY, {0.8});

    std::cout << "MIN_X: " << MIN_X << std::endl;
    std::cout << "MAX_X: " << MAX_X << std::endl;
    std::cout << "DELTA_X: " << DELTA_X << std::endl;
    std::cout << "MAX_Y: " << MAX_Y << std::endl;
    std::cout << "DELTA_Y: " << DELTA_Y << std::endl;
    std::cout << "MAX_YAW: " << MAX_YAW << std::endl;
    std::cout << "DELTA_YAW: " << DELTA_YAW << std::endl;
    std::cout << "LOOKUP_TABLE_FILE_NAME: " << LOOKUP_TABLE_FILE_NAME << std::endl;
    std::cout << "MAX_ACCELERATION: " << MAX_ACCELERATION << std::endl;
    std::cout << "TARGET_VELOCITY: " << TARGET_VELOCITY << std::endl;
}

std::string LookupTableGenerator::process(void)
{
    const int N_X = (MAX_X - MIN_X) / DELTA_X + 1;
    if(N_X < 1){
        std::cout << "param error(x)" << std::endl;
        exit(-1);
    }
    const int N_Y = 2 * MAX_Y / DELTA_Y + 1;
    if(N_Y < 1){
        std::cout << "param error(y)" << std::endl;
        exit(-1);
    }
    const int N_YAW = 2 * MAX_YAW / DELTA_YAW + 1;
    if(N_YAW < 1){
        std::cout << "param error(yaw)" << std::endl;
        exit(-1);
    }
    const int N = N_X * N_Y * N_YAW;
    std::cout << "N_X: " << N_X << std::endl;
    std::cout << "N_Y: " << N_Y << std::endl;
    std::cout << "N_YAW: " << N_YAW << std::endl;
    std::vector<Eigen::Vector3d> states;
    for(int i=0;i<N_YAW;i++){
        for(int j=0;j<N_Y;j++){
            for(int k=0;k<N_X;k++){
                Eigen::Vector3d state;
                state << MIN_X + DELTA_X * k,
                         -MAX_Y + DELTA_Y * j,
                         -MAX_YAW + DELTA_YAW * i;
                states.push_back(state);
            }
        }
    }
    std::cout << "states num: " << states.size() << std::endl;

    std::vector<std::vector<double> > lookup_table_data_list;
    lookup_table_data_list.resize(N);
    for(auto& lookup_table_data : lookup_table_data_list){
        // x, y, yaw, km, kf, sf
        lookup_table_data.resize(6);
    }

    std::string output_data = "x, y, yaw, km, kf, sf\n";
    for(auto state : states){
        std::cout << "state:" << std::endl;
        std::cout << state << std::endl;
        double distance = state.segment(0, 2).norm();
        std::cout << "distance: " << distance << std::endl;
        double v0 = 0.5;// temp
        MotionModelDiffDrive::VelocityParams init_v(v0, MAX_ACCELERATION, TARGET_VELOCITY, TARGET_VELOCITY, MAX_ACCELERATION);
        MotionModelDiffDrive::ControlParams init(init_v, MotionModelDiffDrive::CurvatureParams(0, 0, 0, distance));
        MotionModelDiffDrive::ControlParams output;
        MotionModelDiffDrive::Trajectory trajectory;
        TrajectoryGeneratorDiffDrive tg;
        double cost = tg.generate_optimized_trajectory(state, init, 1e-1, 1e-1, 100, output, trajectory);
        if(cost > 0){
            std::cout << "successfully optimized" << std::endl;
            std::stringstream data;
            data << trajectory.trajectory.back()(0) << "," << trajectory.trajectory.back()(1) << "," << trajectory.trajectory.back()(2) << "," << output.curv.km << "," << output.curv.kf << "," << output.curv.sf << "\n";
            output_data += data.str();
        }else{
            std::cout << "failed to optimize trajectory" << std::endl;
        }
    }
    return output_data;
}

void LookupTableGenerator::save(std::string& data)
{
    std::ofstream ofs(LOOKUP_TABLE_FILE_NAME);
    if(ofs){
        ofs << data;
        ofs.close();
    }else{
        std::cout << "cannot open file" << std::endl;
        exit(-1);
    }
}
