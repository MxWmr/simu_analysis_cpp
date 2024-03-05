#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm> 

// include other files
#include "imu_process.hpp"
#include "utils.hpp"

int main(){

    // Parameters of the analysis
    std::string simu_shape = "LINE"; // "LINE" "TEST" "SQUARE" "8TURN" "HELICOIDAL" "TRIANGLE"
    std::string noise = "0"; // "1" for noise
    std::string simu_name = "SINS_CPP_"+simu_shape+"/"+"SIM_"+noise+"/";
    std::string path = "/home/maxwmr/Documents/work/data2/";
    bool inert = true;
    bool export_tofile = true;





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



    


}