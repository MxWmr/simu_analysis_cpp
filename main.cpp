#include <iostream>
#include <vector>
#include <string>
#include <ceres/ceres.h>
#include <matplot/matplot.h>
#include <cmath>
#include <algorithm> 

// include other files
#include "imu_process.hpp"
#include "utils.hpp"

int main(){

    // Parameters of the analysis
    std::string simu_shape = "TEST"; // "LINE" "TEST" "SQUARE" "8TURN" "HELICOIDAL" "TRIANGLE"
    std::string noise = "0"; // "1" for noise
    std::string simu_name = "SINS_"+simu_shape+"/"+"SIMU_"+noise+"/";
    std::string path = "/home/maxwmr/Documents/work/data"
    bool export = false;





    imu_process process(path,simu_name);

    // get data from files
    process.get_data();

    //orient DVL and IMU in navigtation referential
    process.orient_dvl();
    process.orient_imu();

    //integrate DVL and IMU to get speed and position
    process.integrate_dvl();
    process.integrate_imu1();
    process.integrate_imu2();


    

    // export data to CSV files
    if(export){
        process.export_results();
    }



    


}