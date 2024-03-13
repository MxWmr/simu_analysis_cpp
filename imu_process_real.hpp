#ifndef DEF_IMU_PROCESS_REAL
#define DEF_IMU_PROCESS_REAL
#include <vector>
#include <thread>
#include "utils.hpp"
#include <iostream>
#include <string>
#include "DataWriter.hpp"
#include <ceres/ceres.h>

// typedef std::vector<double> Timevect;
// typedef std::vector<Eigen::Vector3d> Vect3;
// Residual fonctor of USBL



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
    void remove_bias();


    template<typename T>
    Eigen::Vector3<T> process_imu(const Eigen::Vector3<T>& bias,const T scale_factor){

        // remove bias and scale factor
        std::vector<Eigen::Vector3<T>> imuaccel(m_imuaccel0.size());
        for (int i=0;i<m_imutime.size();i++){
            Eigen::Vector3d v = m_dvlspeed_imutime[i];
            double h = -m_depth_imutime[i]; // - because depth is not altitude !!
            double Lat = m_phinslat_imutime[i]*EIGEN_PI/180.;

            Eigen::Matrix3<T> orient_mat = utils::get_rotmat(m_orientation_imutime[i]);
            imuaccel[i]=Eigen::Vector3<T>{scale_factor*m_imuaccel0[i]*50.*1e-7-bias}+ utils::get_g(Lat,h);
        }

        // orientation
        // for (int i=0;i<m_imuaccel.size();i++){

        //     Eigen::Vector3d v = m_dvlspeed_imutime[i];
        //     double h = -m_depth_imutime[i]; // - because depth is not altitude !!
        //     double Lat = m_phinslat_imutime[i]*EIGEN_PI/180.;

        //     Eigen::Matrix3<T> orient_mat = utils::get_rotmat(m_orientation_imutime[i]);
        //     // TO COMPLETE
        //     if (false){
        //         Eigen::Vector3d wie = utils::get_wie(Lat,h);
        //         Eigen::Vector3d wen = utils::get_wen(Lat,v,h);
        //         imuaccel[i] = orient_mat * (imuaccel[i]*50.*1e-7) + utils::get_local_gravity(Lat,h,wie) - (2*wie+wen).cross(v);  
        //     }
        //     else{
        //         imuaccel[i] =  orient_mat * imuaccel[i]*50.*1e-7 + utils::get_g(Lat,h);
        //     }
        // }

        // integration
        Eigen::Vector3<T> v_0 = Eigen::Vector3<T>{m_initspeed};
        std::vector<Eigen::Vector3<T>>  imuspeed;
        imuspeed.push_back(v_0);
        imuspeed.push_back(imuspeed[0] + (m_imutime[1]-m_imutime[0]) * m_imuaccel[1]);
        for (int i=2;i<m_imuaccel.size();i++){
            imuspeed.push_back(imuspeed[i-1] + (m_imutime[i]-m_imutime[i-1])*(m_imuaccel[i]+m_imuaccel[i-1])*0.5); 
        }

        //rmse
        std::vector<Eigen::Vector3<T>> imuspeed_dvltime = utils::interpolateVector(imuspeed,m_imutime,m_dvltime);
        
        Eigen::Vector3<T> rmse = {T{0},T{0},T{0}};
        for (int i=0;i<imuspeed.size();i++){
            rmse= rmse + (imuspeed[i]-m_dvlspeed[i]).array().square().matrix();
        }
        rmse = rmse / T{imuspeed.size()};
        rmse = rmse.array().sqrt().matrix();
        return rmse;



    }


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
    std::vector<Eigen::Vector3d> m_imuaccel0;
    std::vector<Eigen::Vector3d> m_imuspeed;
    std::vector<Eigen::Vector3d> m_imupos;

    std::vector<double>  m_dvltime;
    std::vector<double>  m_depthtime;
    std::vector<double>  m_imutime;
    std::vector<double>  m_usbltime;
    std::vector<double>  m_phinstime;
    std::vector<double>  m_ortime;



};

struct Residual{
    Residual(imu_process_real& obj){
        // m_process = obj;
    }

    template <typename T>
    bool operator()(const T *const bias, const T *const scale_fact, T *residual) const{

		Eigen::Vector3<T> _bias = Eigen::Vector3<T>(bias);
		T _scale_fact = scale_fact[0];


        residual[0] = m_process->process_imu(_bias,_scale_fact)[0];
        residual[1] = m_process->process_imu(_bias,_scale_fact)[1];
        residual[2] = m_process->process_imu(_bias,_scale_fact)[2];

        // residual[0] = T{m_process->dvlimu_rmse()};

        return true;
    }
    private:
    imu_process_real* m_process;

};


#endif