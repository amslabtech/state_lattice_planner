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

}

void LookupTableGenerator::process(void)
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

    std::string output_data;
    for(auto state : states){
        std::cout << "state:" << std::endl;
        std::cout << state << std::endl;
        MotionModelDiffDrive::VelocityParams init_v(0.5, 0);
        double distance = sqrt(state(0)*state(0)+state(1)*state(1));
        std::cout << "distance: " << distance << std::endl;
        MotionModelDiffDrive::CurvatureParams init_c(0, 0, 0, distance);
        MotionModelDiffDrive::VelocityParams output_v;
        MotionModelDiffDrive::CurvatureParams output_c;
        std::vector<Eigen::Vector3d> trajectory;
        TrajectoryGeneratorDiffDrive tg;
        tg.set_param(0.005, 0.005, 0.5);

        double cost = tg.generate_optimized_trajectory(state, init_v, init_c, 0.05, 0.1, 100, output_v, output_c, trajectory);
        if(cost > 0){
            std::cout << "successfully optimized" << std::endl;
            std::stringstream data;
            data << trajectory.back()(0) << "," << trajectory.back()(1) << "," << trajectory.back()(2) << "," << output_c.km << "," << output_c.kf << "," << output_c.sf << "\n";
            output_data += data.str();
        }else{
            std::cout << "failed to optimize trajectory" << std::endl;
        }
    }
    std::ofstream ofs(LOOKUP_TABLE_FILE_NAME);
    if(ofs){
        ofs << output_data;
        ofs.close();
    }else{
        std::cout << "cannot open file" << std::endl;
        exit(-1);
    }
}