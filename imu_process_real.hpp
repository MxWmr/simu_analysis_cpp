#pragma once
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
    std::vector<double> get_imutime();
    std::vector<double> get_dvltime();
    std::vector<double> get_depth_imutime();
    std::vector<double> get_phinslat_imutime();
    std::vector<Eigen::Vector3d> get_orientation_imutime();
    std::vector<Eigen::Vector3d> get_imuaccel0();
    std::vector<Eigen::Vector3d> get_dvlspeed();
    Eigen::Vector3d get_initspeed();
    void remove_bias();


    template<typename T>
    Eigen::Vector3<T> process_imu(const Eigen::Vector3<T>& bias,const T scale_factor,const std::vector<double>& imutime,const std::vector<double>& dvltime,const std::vector<double>& depth_imutime,
                                const std::vector<double>& phinslat_imutime, const std::vector<Eigen::Vector3d>&  orientation_imutime, const std::vector<Eigen::Vector3d>& imuaccel0, const std::vector<Eigen::Vector3d>& dvlspeed,
                                const Eigen::Vector3d& initspeed){
        int N = imutime.size();
        // remove bias and scale factor
        std::vector<Eigen::Vector3<T>> imuaccel;
        for (int i=0;i<N;i++)
        {

            double h = -depth_imutime[i]; // - because depth is not altitude !!
            double Lat = phinslat_imutime[i]*EIGEN_PI/180.;

            Eigen::Matrix3d orient_mat = utils::get_rotmat(orientation_imutime[i]);
            orient_mat.transposeInPlace();

            imuaccel.push_back(Eigen::Vector3<T>{scale_factor*imuaccel0[i]*50.*1e-7-bias} + orient_mat*utils::get_g(Lat,h));
        }

        // // orientation
        // // for (int i=0;i<m_imuaccel.size();i++){

        // //     Eigen::Vector3d v = m_dvlspeed_imutime[i];
        // //     double h = -m_depth_imutime[i]; // - because depth is not altitude !!
        // //     double Lat = m_phinslat_imutime[i]*EIGEN_PI/180.;

        // //     Eigen::Matrix3<T> orient_mat = utils::get_rotmat(m_orientation_imutime[i]);
        // //     // TO COMPLETE
        // //     if (false){
        // //         Eigen::Vector3d wie = utils::get_wie(Lat,h);
        // //         Eigen::Vector3d wen = utils::get_wen(Lat,v,h);
        // //         imuaccel[i] = orient_mat * (imuaccel[i]*50.*1e-7) + utils::get_local_gravity(Lat,h,wie) - (2*wie+wen).cross(v);  
        // //     }
        // //     else{
        // //         imuaccel[i] =  orient_mat * imuaccel[i]*50.*1e-7 + utils::get_g(Lat,h);
        // //     }
        // // }

        // integration
        Eigen::Vector3<T> v_0 = {T{initspeed[0]},T{initspeed[1]},T{initspeed[2]}};
        std::vector<Eigen::Vector3<T>>  imuspeed;
        imuspeed.push_back(v_0);
        for (int i=1;i<N;i++){
            imuspeed.push_back(imuspeed[i-1] + (imutime[i]-imutime[i-1])*imuaccel[i]); 
        }

        // //rmse
        
         Eigen::Vector3<T> rmse = {T{0},T{0},T{0}};
        for (int i=0;i<N;i++){
            rmse= rmse + (imuspeed[i]-m_dvlspeed_imutime[i]).array().square().matrix();
        }
        double len = 1/N;
        rmse = rmse / len;
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
    std::vector<double>  m_imutime = {0};
    std::vector<double>  m_usbltime;
    std::vector<double>  m_phinstime;
    std::vector<double>  m_ortime;



};


struct Res_bias_sf{
    Res_bias_sf(imu_process_real& obj){m_process = &obj;};
    template <typename T>
    bool operator()(const T *const bias, const T *const scale_fact, T *residual) const{

		Eigen::Vector3<T> _bias = Eigen::Vector3<T>(bias);
		T _scale_fact = scale_fact[0];

        Eigen::Vector3<T> werr;
        werr = m_process->process_imu(_bias,_scale_fact, m_process->get_imutime(), m_process->get_dvltime(),
                                     m_process->get_depth_imutime(), m_process->get_phinslat_imutime(), 
                                     m_process->get_orientation_imutime(), m_process->get_imuaccel0(), 
                                     m_process->get_dvlspeed(), m_process->get_initspeed());

        residual[0] = werr[0];
        residual[1] = werr[1];
        residual[2] = werr[2];
        // residual[0] = werr.array().mean();
        return true;
    }
    private:
    imu_process_real* m_process;

};


