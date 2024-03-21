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

class imu_process
{
public:
    imu_process(std::string path, std::string simu_name);
    void get_data(const bool simu);
    void orient_dvl(const bool simu);
    void orient_imu(const bool inert = false,const Eigen::Vector3d& bias=Eigen::Vector3d::Zero(), const double scale_factor=1);
    void integrate_dvl();
    void integrate_imu1();
    void integrate_imu2();
    void export_results(const bool simu);
    std::pair<int, int> cutTime(std::vector<double> &time, const double &beginTime, const double &endTime = INFINITY);
    void keepDataInSameInterval();
    std::vector<double> rmse();
    std::vector<double> get_imutime();
    std::vector<double> get_dvltime();
    std::vector<double> get_depth_imutime();
    std::vector<double> get_phinslat_imutime();
    std::vector<Eigen::Vector3d> get_orientation_imutime();
    std::vector<Eigen::Vector3d> get_imuaccel_b();
    std::vector<Eigen::Vector3d> get_dvlspeed_n();
    Eigen::Vector3d get_initspeed();
    Eigen::Vector3d get_bias();
    double get_scale_factor();
    void find_bias();

    template <typename T>
    void cutData(std::vector<T> &data, const std::pair<int, int> &interval)
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
    Eigen::Matrix<double, 3, 3> m_misalignement_dvl2b;
    double m_scale_factor;
    Eigen::Vector3d m_bias;

    std::vector<double> m_depth;
    std::vector<double> m_depth_imutime;
    std::vector<double> m_usbllat;
    std::vector<Eigen::Vector3d> m_usblgeo;

    std::vector<Eigen::Vector3d> m_orientation_b2n;
    std::vector<Eigen::Vector3d> m_orientation_b2n_dvltime;
    std::vector<Eigen::Vector3d> m_orientation_b2n_imutime;

    std::vector<Eigen::Vector3d> m_phinspos;
    std::vector<Eigen::Vector3d> m_phinspos_dvltime;
    std::vector<Eigen::Vector3d> m_phinspos_imutime;
    std::vector<double> m_phinslat;
    std::vector<double> m_phinslat_imutime;

    std::vector<Eigen::Vector3d> m_dvlspeed_b;
    std::vector<Eigen::Vector3d> m_dvlspeed_n;
    std::vector<Eigen::Vector3d> m_dvlspeed_n_imutime;
    std::vector<Eigen::Vector3d> m_dvlpos;

    std::vector<Eigen::Vector3d> m_imuaccel_b;
    std::vector<Eigen::Vector3d> m_imuaccel_n;
    std::vector<Eigen::Vector3d> m_imuspeed_n;
    std::vector<Eigen::Vector3d> m_imupos;

    std::vector<Eigen::Vector3d> m_refpos;
    std::vector<Eigen::Vector3d> m_refspeed_n;
    std::vector<Eigen::Vector3d> m_refspeed_n_imutime;    
    std::vector<Eigen::Vector3d> m_refpos_imutime;
    std::vector<Eigen::Vector3d> m_refspeed_n_dvltime;
    std::vector<Eigen::Vector3d> m_refpos_dvltime;
    std::vector<double> m_reflat;
    std::vector<double> m_refdepth;
    std::vector<double> m_refdepth_imutime;
    std::vector<double> m_reflat_imutime;



    std::vector<double> m_dvltime;
    std::vector<double> m_depthtime;
    std::vector<double> m_imutime;
    std::vector<double> m_usbltime;
    std::vector<double> m_phinstime;
    std::vector<double> m_ortime;
    std::vector<double> m_reftime;
};

struct Res_bias_sf
{
    Res_bias_sf(imu_process &obj, int i)
    {
        m_process = &obj;
        m_i = i;
    };
    template <typename T>
    bool operator()(const T *const bias, const T *const scale_fact, T *residual) const
    {

        Eigen::Vector3<T> _bias = Eigen::Vector3<T>(bias);
        T _scale_fact = scale_fact[0];

        std::vector<Eigen::Vector3d> dvlspeed_n = m_process->get_dvlspeed_n();
        std::vector<Eigen::Vector3d> imuaccel_b = m_process->get_imuaccel_b();
        std::vector<Eigen::Vector3d> orientation_b2n_imutime = m_process->get_orientation_imutime();
        std::vector<double> imutime = m_process->get_imutime();
        std::vector<double> dvltime = m_process->get_dvltime();
        std::vector<double> depth_imutime = m_process->get_depth_imutime();
        std::vector<double> phinslat_imutime = m_process->get_phinslat_imutime();

        // Eigen::Vector3<T> werr;
        Eigen::Vector3<T> werr = {T{0},T{0},T{0}};

        int j(0);
        while (imutime[j] <= dvltime[m_i - 1]){j++;}

        Eigen::Vector3<T> speedincr;
        while (imutime[j] < dvltime[m_i] && j < imutime.size())
        {

            double h = -depth_imutime[j]; // - because depth is not altitude !!
            double Lat = phinslat_imutime[j] * PI / 180.;

            Eigen::Matrix3d R_b2n = utils::get_rotmat(orientation_b2n_imutime[j]);

            speedincr = 0.02 * (Eigen::Vector3<T>{R_b2n * ((imuaccel_b[j]  - _bias) / _scale_fact)} + utils::get_g(Lat, h));
            werr -= speedincr;

            j++;
        }
        werr += dvlspeed_n[m_i] - dvlspeed_n[m_i - 1];

        if (m_i % 100 == 0)
        {
            // std::cout << werr << std::endl;
        }
        residual[0] = werr[0];
        residual[1] = werr[1];
        residual[2] = werr[2];
        return true;
    }

private:
    imu_process *m_process;
    int m_i;
};




struct Res_cstbias
{

    Res_cstbias(const double std):m_std(std){}

    template <typename T>
    bool operator()(const T *const biasi, const T *const biasi_1, T *residual) const
    {
        Eigen::Vector3<T> _biasi = Eigen::Vector3<T>(biasi);
        Eigen::Vector3<T> _biasi_1 = Eigen::Vector3<T>(biasi_1);

        Eigen::Vector3<T> werr = {T{0},T{0},T{0}};

        werr += 1./m_std*(_biasi-_biasi_1);

        residual[0] = werr[0];
        residual[1] = werr[1];
        residual[2] = werr[2];

        return true;
    }

private:
    double m_std;
};
