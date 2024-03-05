#ifndef DEF_IMU_PROCESS
#define DEF_IMU_PROCESS
#include <vector>
#include <thread>
#include "utils.hpp"
#include <iostream>
#include <string>
#include "DataWriter.hpp"

// typedef std::vector<double> Timevect;
// typedef std::vector<Eigen::Vector3d> Vect3;


class imu_process{
    public:
    imu_process(std::string path, std::string simu_name);
    void get_data();
    void orient_dvl();
    void orient_imu(const bool inert=false);
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
    Eigen::Matrix<double, 3, 3> m_misalignement;

    std::vector<double> m_depth;
    std::vector<double> m_depth_dvltime;
    std::vector<double> m_usbllat;

    std::vector<Eigen::Vector3d> m_orientation;
    std::vector<Eigen::Vector3d> m_orientation_dvltime;
    std::vector<Eigen::Vector3d> m_orientation_imutime;

    std::vector<Eigen::Vector3d> m_refpos;
    std::vector<Eigen::Vector3d> m_refspeed;
    std::vector<Eigen::Vector3d> m_refspeed_imutime;    
    std::vector<Eigen::Vector3d> m_refpos_imutime;
    std::vector<Eigen::Vector3d> m_refspeed_dvltime;
    std::vector<Eigen::Vector3d> m_refpos_dvltime;
    std::vector<double> m_reflat;
    std::vector<double> m_refdepth;
    std::vector<double> m_refdepth_imutime;
    std::vector<double> m_reflat_imutime;

    std::vector<Eigen::Vector3d> m_dvlspeed;
    std::vector<Eigen::Vector3d> m_dvlpos;

    std::vector<Eigen::Vector3d> m_imuaccel;
    std::vector<Eigen::Vector3d> m_imuspeed;
    std::vector<Eigen::Vector3d> m_imupos;

    std::vector<double>  m_dvltime;
    std::vector<double>  m_depthtime;
    std::vector<double>  m_imutime;
    std::vector<double>  m_usbltime;
    std::vector<double>  m_reftime;
    std::vector<double>  m_ortime;



};

#endif