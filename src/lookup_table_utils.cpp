#include "state_lattice_planner/lookup_table_utils.h"

namespace LookupTableUtils
{
    StateWithControlParams::StateWithControlParams(void)
    {

    }

    bool load_lookup_table(const std::string& lookup_table_file_name, LookupTable& lookup_table)
    {
        lookup_table.clear();
        std::cout << "loading lookup table from " << lookup_table_file_name << std::endl;
        std::ifstream ifs(lookup_table_file_name);
        if(ifs){
            bool first_line_flag = true;
            while(!ifs.eof()){
                std::string data;
                std::getline(ifs, data);
                if(data == ""){
                    continue;
                }
                if(first_line_flag){
                    first_line_flag = false;
                    continue;
                }
                std::istringstream stream(data);
                std::vector<double> splitted_data;
                std::string buffer;
                while(std::getline(stream, buffer, ',')){
                    splitted_data.push_back(std::stod(buffer));
                }
                StateWithControlParams param;
                auto it = splitted_data.begin();
                double v0 = *(it);
                param.control.omega.k0 = *(++it);
                double x = *(++it);
                double y = *(++it);
                double yaw = *(++it);
                param.state << x, y, yaw;
                param.control.omega.km = *(++it);
                param.control.omega.kf = *(++it);
                param.control.omega.sf = *(++it);
                lookup_table[v0][param.control.omega.k0].push_back(param);
            }
            ifs.close();
        }else{
            std::cout << "\033[91mERROR: cannot open file\033[00m" << std::endl;
            // exit(-1);
            return false;
        }
        return true;
    }

    void get_optimized_param_from_lookup_table(const LookupTable& lookup_table, const Eigen::Vector3d goal, const double v0, const double k0, MotionModelDiffDrive::ControlParams& param)
    {
        if(lookup_table.size() > 0){
            double min_v_diff = 1e3;
            double v = 0;
            for(const auto& v_data : lookup_table){
                double _v = v_data.first;
                double diff = fabs(_v - v0);
                if(diff < min_v_diff){
                    min_v_diff = diff;
                    v = _v;
                }
            }
            double min_k_diff = 1e3;
            double k = 0;
            for(const auto& k_data : lookup_table.at(v)){
                double _k = k_data.first;
                double diff = fabs(_k - k0);
                if(diff < min_k_diff){
                    min_k_diff = diff;
                    k = _k;
                }
            }
            double min_cost = 1e3;
            StateWithControlParams _param;
            for(const auto& data : lookup_table.at(v).at(k)){
                // sqrt(x^2 + y^2 + yaw^2)
                double cost = (goal - data.state).norm();
                if(cost < min_cost){
                    min_cost = cost;
                    _param = data;
                }
            }
            param = _param.control;
        }else{
            param.omega.km = 0;
            param.omega.kf = 0;
            param.omega.sf = goal.segment(0, 2).norm();
        }
    }
}
