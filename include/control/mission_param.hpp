#ifndef GAIN_TUNING_HPP
#define GAIN_TUNING_HPP

#include <fstream>
#include <vector>
#include <nlohmann/json.hpp>
#include <libgen.h>
#include <unistd.h>
#include <stdexcept> // for std::runtime_error

#include "callback_data_manage.hpp"
#include "lat_control.hpp"
#include "lon_control.hpp"
#include "control/kalman_filter.hpp"

using json = nlohmann::json;

std::string file_name = "A";
// std::string file_name = "B";  // 새로운 튜닝값을 쓰고 싶으면 B.json 파일을 만든 뒤 같은 형식으로 사용하면 됨.

class ControlGainTuning
{
private:
    CallbackClass *cb_data_;
    CombinedSteer *lat_con_;
    LonController *long_con_;
    KalmanFilter2D *km_filter_;

    json file_data;


public:
    ControlGainTuning(CallbackClass *cb_data, CombinedSteer *lat_con, LonController *long_con, KalmanFilter2D *km_filter)
    {
        this->cb_data_ = cb_data;
        this->lat_con_ = lat_con;
        this->long_con_ = long_con;
        this->km_filter_ = km_filter;

        read_json();
        // TODO: pp에 min, max LD 값 Json으로 읽어오기.
    }

    int read_json()
    {
        char path[1024];
        realpath(dirname(strdup(__FILE__)), path);
        std::string dir_path_str(path);

        std::string file_path = dir_path_str + "/" + file_name + ".json";

        // JSON 파일 읽기
        std::ifstream input_file(file_path);

        if (!input_file.is_open())
        {
            std::cerr << "Error opening JSON file: " << file_path << std::endl;
            return 1;
        }

        // JSON 파싱
        json data;
        try
        {
            input_file >> data;
        }
        catch (const json::parse_error &e)
        {
            std::cerr << "JSON parse error: " << e.what() << std::endl;
            input_file.close();
            return 1;
        }

        this->file_data = data;


        return 0;
    }

    void set_normal_param()
    {
        json normal_data = file_data["normal"];

        // longitudinal param
        json lon_param = normal_data["______lllllllllllllll______"];
        long_con_->set_lon_PD_gain(lon_param["PD_gas_gain"][0], lon_param["PD_gas_gain"][1]);
        long_con_->set_lon_target_speed(lon_param["target_speed"]);


        // latarl param
        json lat_param = normal_data["______---------------______"];
        // stanly------------------------------------------------------------------------
        lat_con_->set_stanly_gain(lat_param["stanly_gain"][0], lat_param["stanly_gain"][1], lat_param["stanly_gain"][2]);
        lat_con_->set_heading_gain(lat_param["stanly_heading_gain"]);
        lat_con_->set_anti_windup_max(lat_param["anti_windup_val"]);

        // kimm lateral param
        json kimm_lat_param = normal_data["______kimm______"];
        // stanly------------------------------------------------------------------------
        lat_con_->set_kimm_gain(kimm_lat_param["kimm_gain"]);
        lat_con_->set_kimm_heading_gain(kimm_lat_param["kimm_heading_gain"]);
        lat_con_->set_kimm_hd_rangge_div(kimm_lat_param["kimm_hd_rangge_div"]);  
        lat_con_->set_kimm_docking_gain(kimm_lat_param["kimm_docking_gain"][0],kimm_lat_param["kimm_docking_gain"][1], \
            kimm_lat_param["kimm_docking_gain"][2],kimm_lat_param["kimm_docking_gain"][3],kimm_lat_param["kimm_docking_gain"][4], \
            kimm_lat_param["kimm_docking_gain"][5],kimm_lat_param["kimm_docking_gain"][6],kimm_lat_param["kimm_docking_gain"][7], \
            kimm_lat_param["kimm_docking_gain"][7]); 
        lat_con_->set_docking_anti_windup_max(kimm_lat_param["kimm_docking_anti_wind_up"][0],kimm_lat_param["kimm_docking_anti_wind_up"][1],\
            kimm_lat_param["kimm_docking_anti_wind_up"][2]);  
        //kalman filter
        json km_param = normal_data["______kkkkkkkkkkkkkkk______"];
        // stanly------------------------------------------------------------------------
        km_filter_->set_Q_value(km_param["Q"][0], km_param["Q"][1]);
        km_filter_->set_R_value(km_param["R"][0], km_param["R"][1], km_param["R"][2]);
        // km_filter_->set_P_value(km_param["P"][0], km_param["P"][1]);
    }

    


};

#endif // GAIN_TUNING_HPP
