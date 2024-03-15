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

class imu_process_real
{
public:
    imu_process_real(std::string path, std::string simu_name);
    void get_data();
    void orient_dvl();
    void orient_imu(const bool inert = false);
    void integrate_dvl();
    void integrate_imu1();
    void integrate_imu2();
    void export_results();
    std::pair<int, int> cutTime(std::vector<double> &time, const double &beginTime, const double &endTime = INFINITY);
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

    std::vector<double> m_dvltime;
    std::vector<double> m_depthtime;
    std::vector<double> m_imutime = {0};
    std::vector<double> m_usbltime;
    std::vector<double> m_phinstime;
    std::vector<double> m_ortime;
};

struct Res_bias_sf
{
    Res_bias_sf(imu_process_real &obj, int i)
    {
        m_process = &obj;
        m_i = i;
    };
    template <typename T>
    bool operator()(const T *const bias, const T *const scale_fact, T *residual) const
    {

        Eigen::Vector3<T> _bias = Eigen::Vector3<T>(bias);
        T _scale_fact = scale_fact[0];

        std::vector<Eigen::Vector3d> dvlspeed = m_process->get_dvlspeed();
        std::vector<Eigen::Vector3d> imuaccel0 = m_process->get_imuaccel0();
        std::vector<Eigen::Vector3d> orientation_imutime = m_process->get_orientation_imutime();
        std::vector<double> imutime = m_process->get_imutime();
        std::vector<double> dvltime = m_process->get_dvltime();
        std::vector<double> depth_imutime = m_process->get_depth_imutime();
        std::vector<double> phinslat_imutime = m_process->get_phinslat_imutime();

        // Eigen::Vector3<T> werr;
        Eigen::Vector3<T> werr;

        int j(0);
        while (imutime[j] < dvltime[m_i - 1])
        {
            j++;
        }

        Eigen::Vector3<T> speedincr;
        while (imutime[j] <= dvltime[m_i] && j < imutime.size())
        {

            double h = -depth_imutime[j]; // - because depth is not altitude !!
            double Lat = phinslat_imutime[j] * PI / 180.;

            Eigen::Matrix3d R_b2n = utils::get_rotmat(orientation_imutime[j]);

            speedincr = 0.02 * (Eigen::Vector3<T>{R_b2n * (imuaccel0[j] / _scale_fact - _bias)} + utils::get_g(Lat, h));
            werr[0] -= speedincr[0];
            werr[1] -= speedincr[1];
            werr[2] -= speedincr[2];
            j++;
        }
        werr += dvlspeed[m_i] - dvlspeed[m_i - 1];

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
    imu_process_real *m_process;
    int m_i;
};
