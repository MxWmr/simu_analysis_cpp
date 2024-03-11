#ifndef DEF_IMU_PROCESS_REAL
#define DEF_IMU_PROCESS_REAL
#include <vector>
#include <thread>
#include "utils.hpp"
#include <iostream>
#include <string>
#include "DataWriter.hpp"

// typedef std::vector<double> Timevect;
// typedef std::vector<Eigen::Vector3d> Vect3;


class imu_process_real{
    public:
    imu_process_real(std::string path, std::string simu_name);
    void get_data();
    void orient_dvl();
    void orient_imu(const bool inert=false);
    void integrate_dvl();
    void integrate_imu1();
    void integrate_imu2();
    void export_results();
	std::pair<int, int> cutTime(std::vector<double>& time, const double& beginTime, const double& endTime = INFINITY);
	void keepDataInSameInterval();



	template<typename T>
	void cutData(std::vector<T>& data, const std::pair<int, int>& interval)
	{
		int begin = interval.first;
		int end = interval.second;

		std::vector<T> newData;

		for (int i(begin); i <= end; i++)
		{
			newData.push_back(data[i]);
		}

		newData.swap(data);
	}


    protected:
    std::string m_path;
    std::string m_simu_name;
    Eigen::Vector3d m_initpos;
    Eigen::Vector3d m_initspeed;
    Eigen::Matrix<double, 3, 3> m_misalignement;

    std::vector<double> m_depth;
    std::vector<double> m_depth_imutime;
    std::vector<double> m_usbllat;
    std::vector<Eigen::Vector3d> m_usblgeo;

    std::vector<Eigen::Vector3d> m_orientation;
    std::vector<Eigen::Vector3d> m_orientation_dvltime;
    std::vector<Eigen::Vector3d> m_orientation_imutime;

    std::vector<Eigen::Vector3d> m_phinspos; 
    std::vector<Eigen::Vector3d> m_phinsspeed_dvltime;
    std::vector<Eigen::Vector3d> m_phinspos_dvltime;
    std::vector<double> m_phinslat;
    std::vector<double> m_phinslat_imutime;

    std::vector<Eigen::Vector3d> m_dvlspeed;
    std::vector<Eigen::Vector3d> m_dvlspeed_imutime;
    std::vector<Eigen::Vector3d> m_dvlpos;

    std::vector<Eigen::Vector3d> m_imuaccel;
    std::vector<Eigen::Vector3d> m_imuspeed;
    std::vector<Eigen::Vector3d> m_imupos;

    std::vector<double>  m_dvltime;
    std::vector<double>  m_depthtime;
    std::vector<double>  m_imutime;
    std::vector<double>  m_usbltime;
    std::vector<double>  m_phinstime;
    std::vector<double>  m_ortime;



};

#endif