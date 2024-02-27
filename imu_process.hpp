#ifndef DEF_IMU_PROCESS
#define DEF_IMU_PROCESS
#include <vector>
#include <thread>
#include <ceres/ceres.h>
#include "utils.hpp"


typedef std::vector<Eigen::Vector3d> Vect3;
typedef std::vector<std::time_t> Timevect;


class imu_process{
    public:
    void imu_process(std::string path, std::string simu_name);
    void get_data();
    void orient_dvl();
    void orient_imu();
    void integrate_dvl();
    void integrate_imu1();
    void integrate_imu2();
    std::vector<double> imu_rmse();
    std::vector<double> dvl_rmse();
    void export_results();


    protected:
    std::string m_path;
    std::string m_simu_name;
    Eigen::Vector3d m_initpos;
    Eigen::Vector3d m_initspeed;

    std::vector<double> m_depth;
    std::vector<double> m_usbllat;

    Vect3 m_refspeed_imutime;    
    Vect3 m_refpos_imutime;
    Vect3 m_refspeed_dvltime;
    Vect3 m_refpos_dvltime;

    Vect3 m_dvlspeed;
    Vect3 m_dvlpos;

    Vect3 m_imuaccel;
    Vect3 m_imuspeed;
    Vect3 m_imupos;

    Timevect m_dvltime;
    Timevect m_imutime;
    Timevect m_usbltime;

    Eigen::Matrix<double, 3, 3> m_misalignement;
    Vect3m_orientation_dvltime;
    Vect3m_orientation_imutime;

}

#endif