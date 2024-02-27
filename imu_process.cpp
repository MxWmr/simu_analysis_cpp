#include "imu_process.hpp"
#include <vector>
#include <thread>
#include <ceres/ceres.h>
#include "utils.hpp"

typedef std::vector<Eigen::Vector3d> Vect3;
typedef std::vector<std::time_t> Timevect;


void imu_process::imu_process(std::string path, std::string simu_name){
    m_simu_name = simu_name;
    m_pzth = path;
}

void imu_process::get_data(){
    // TO DO
}

imu_process::orient_dvl(){
    std::cin << "Orient DVL ..."<<std::endl;

    for (int i=0;i<m_dvlspeed.size();i++){
        Eigen::Matrix3d orient_mat = utils::get_rotmat(m_orientation_dvltime[i]);
        m_dvlspeed[i] = m_misalignement * (orient_mat * m_dvlspeed[i]);
    }

    std::cin << "Done !"<<std::endl;
}

void imu_process::orient_imu(){
    std::cin << "Orient IMU ..."<<std::endl;
    int k(0),l(0);
    for (int i=0;i<m_imuaccel.size();i++){
        Eigen::Vector3d v = m_dvlspeed[k];
        double h = m_depth[k];
        double Lat = m_usbllat[l];
        Eigen::Matrix3d orient_mat = utils::get_rotmat(m_orientation_imutime[i]);
        // TO COMPLETE
        Eigen::Matrix3d wie = utils::get_wie(Lat,h);
        Eigen::Matrix3d wen = utils::get_wen(Lat,v,h);
        m_imuaccel[i] = orient_mat * m_imuaccel[i] - utils::get_local_gravity(Lat,h,wie) - (2*wie+wen)*v
        if (m_dvltime[k]>m_imutime[i-1] && m_dvltime[k]<m_imutime[i] && k<m_dvltime.size()-1){
            k+=1;
        }               
        if (m_usbltime[l]>m_imutime[i-1] and m_usbltime[l]<m_imutime[i] and l<m_usbltime.size()-1){
            l+=1;
        }

    }
    std::cin << "Done !"<<std::endl;
}


void imu_process::integrate_dvl(){
    std::cin << "Integrate DVL ..."<<std::endl;

    Eigen::Vector3d p_0 = m_initpos;
    std::vector<Eigen::Vector3d>  dvlpos = m_dvlspeed;
    dvlpos[0] = p_0

    for (int i=1;i<m_imuaccel.size();i++){
        dvlpos[i] = dvlpos[i-1] + (m_dvltime[i]-m_dvltime[i-1])*(m_dvlspeed[i]+m_dvlspeed[i-1])*0.5 
    }

    m_dvlpos = dvlpos;
    std::cin << "Done !"<<std::endl;
}


void imu_process::integrate_imu2(){
    std::cin << "Integrate IMU ..."<<std::endl;

    Eigen::Vector3d v_0 = m_initspeed;
    std::vector<Eigen::Vector3d>  imuspeed = m_imuaccel;
    imupos[0] = p_0

    for (int i=1;i<m_imuaccel.size();i++){
        imuspeed[i] = imuspeed[i-1] + (m_imutime[i]-m_imutime[i-1])*(m_imuaccel[i]+m_imuaccel[i-1])*0.5 
    }

    m_imuspeed = imuspeed;
    std::cin << "Done !"<<std::endl;
}


void imu_process::integrate_imu2(){
    std::cin << "Integrate IMU ..."<<std::endl;

    Eigen::Vector3d p_0 = m_initpos;
    std::vector<Eigen::Vector3d>  imupos = m_imuspeed;
    imupos[0] = p_0

    for (int i=1;i<m_imuaccel.size();i++){
        imupos[i] = imupos[i-1] + (m_imutime[i]-m_imutime[i-1])*(m_imuspeed[i]+m_imuspeed[i-1])*0.5 
    }

    m_imupos = imupos;
    std::cin << "Done !"<<std::endl;
}


std::vector<double>  imu_process::imu_rmse(){
    std::vector<double> res(2);
    
    res[0]=utils::rmse(m_imuspeed,m_refspeed_imutime);
    res[1]=utils::rmse(m_imupos,m_refpos_imutime);

    return res;

}

std::vector<double>  imu_process::dvl_rmse(){
    std::vector<double> res(2);
    
    res[0]=utils::rmse(m_dvlspeed,m_refspeed_dvltime);
    res[1]=utils::rmse(m_dvlpos,m_refpos_dvltime);

    return res;

}

void imu_process::export_results(){
    // TO DO 
}