#ifndef DEF_IMU_PROCESS
#define DEF_IMU_PROCESS
#include <vector>
#include <thread>
#include <ceres/ceres.h>
#include "utils.hpp"

class imu_process{
    public:
    void imu_process(std::string simu_name);
    void get_data();
    void orient_dvl();
    void orient_imu();
    void integrate_dvl();
    void integrate_imu1();
    void integrate_imu2();

    protected:
    std::string m_simu_name;
    Eigen::Vector3d m_initpos;
    Eigen::Vector3d m_initspeed;

    std::vector<double> m_depth;
    std::vector<double> m_usbllat;
    std::vector<Eigen::Vector3d>  m_dvlspeed;
    std::vector<Eigen::Vector3d>  m_dvlpos;
    std::vector<Eigen::Vector3d>  m_imuaccel;
    std::vector<Eigen::Vector3d>  m_imuspeed;
    std::vector<Eigen::Vector3d>  m_imupos;

    std::vector<std::time_t> m_dvltime;
    std::vector<std::time_t> m_imutime;
    std::vector<std::time_t> m_usbltime;

    Eigen::Matrix<double, 3, 3> m_misalignement;
    std::vector<Eigen::Vector3d> m_orientation_dvltime;
    std::vector<Eigen::Vector3d> m_orientation_imutime;

}

#endif