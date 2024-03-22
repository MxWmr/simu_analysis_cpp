#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm> 
#include <ceres/ceres.h>

// include other files
#include "imu_process.hpp"
#include "utils.hpp"

int main(){

    bool simu =true;

    bool optim_bias = true;
    bool inert = false;
    bool export_tofile = true;
    bool res_rmse = false;  // not already finished
    std::string path = "/home/maxwmr/Documents/work/data2/";
    std::string  simu_name;




    if (simu)
    {
        std::string simu_shape = "TEST"; // "LINE" "TEST" "SQUARE" "8TURN" "HELICOIDAL" "TRIANGLE"
        std::string noise = "0"; // "1" for noise
        simu_name = "SINS_CPP_"+simu_shape+"/"+"SIM_"+noise+"/";
    }
    else
    {
        simu_name = "ESSCORAL19_02/";
    }


    imu_process process(path,simu_name);

    // get data from files
    process.get_data(simu);




    //orient DVL and IMU in navigtation referential
    process.orient_dvl(simu);
    std::vector<Eigen::Vector3d> bias_z;
    for (int i(0);i<process.get_imutime().size();i++)
    {
        bias_z.push_back(Eigen::Vector3d::Zero());
    }
    process.orient_imu(inert,bias_z,1.);


    //integrate DVL and IMU to get speed and position
    process.integrate_dvl();
    process.integrate_imu1();
    process.integrate_imu2();

    if (optim_bias){
        process.find_bias();
        std::vector<Eigen::Vector3d> estimated_bias = process.get_bias();
        process.orient_imu(inert,estimated_bias,1.);
        process.integrate_imu1();
        process.integrate_imu2();
    }
    

    // export data to CSV files
    if(export_tofile){
        process.export_results(simu);
    }


    if(res_rmse){

        std::vector<double> imu_r = process.rmse();

        std::cout << "RMSE betwwen imu speed and ref speed: "<<imu_r[0]<<std::endl;
        std::cout << "RMSE betwwen imu pos and ref pos: "<<imu_r[1]<<std::endl;        
    }





    


}