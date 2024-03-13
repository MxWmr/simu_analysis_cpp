#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm> 
#include <ceres/ceres.h>

// include other files
#include "imu_process.hpp"
#include "imu_process_real.hpp"
#include "utils.hpp"

int main(){

    bool simu =false;

    bool optim_bias = true;
    bool inert = false;
    bool export_tofile = true;
    bool res_rmse = true;
    std::string path = "/home/maxwmr/Documents/work/data2/";
    std::string  simu_name;


    if (simu){
        // Parameters of the analysis
        std::string simu_shape = "HELICOID"; // "LINE" "TEST" "SQUARE" "8TURN" "HELICOIDAL" "TRIANGLE"
        std::string noise = "0"; // "1" for noise
        simu_name = "SINS_CPP_"+simu_shape+"/"+"SIM_"+noise+"/";


        imu_process process(path,simu_name);

        // get data from files
        process.get_data();

        //orient DVL and IMU in navigtation referential
        process.orient_dvl();
        process.orient_imu(inert);

        //integrate DVL and IMU to get speed and position
        process.integrate_dvl();
        process.integrate_imu1();
        process.integrate_imu2();


        

        // export data to CSV files
        if(export_tofile){
            process.export_results();
        }


        if(res_rmse){
            std::vector<double> dvl_r = process.dvl_rmse();
            std::vector<double> imu_r = process.imu_rmse();

            std::cout << "RMSE betwwen imu speed and ref speed: "<<imu_r[0]<<std::endl;
            std::cout << "RMSE betwwen imu pos and ref pos: "<<imu_r[1]<<std::endl;
            std::cout << "RMSE betwwen dvl pos and ref pos: "<<dvl_r[1]<<std::endl;
            
        }


    }

    else{
        simu_name = "ESSCORAL19_02/";

        imu_process_real process(path,simu_name);

        // get data from files
        process.get_data();


        if (optim_bias){
            process.remove_bias();
        }

        //orient DVL and IMU in navigtation referential
        process.orient_dvl();
        process.orient_imu(inert);

        //integrate DVL and IMU to get speed and position
        process.integrate_dvl();
        process.integrate_imu1();
        process.integrate_imu2();


        

        // export data to CSV files
        if(export_tofile){
            process.export_results();
        }


    }






    


}