#pragma once
#include <iomanip>
#include <vector>
#include <random>

#include <Eigen/Dense>
#include <matplot/matplot.h>

const double PI = 3.141592653589793;
typedef std::vector<Eigen::Vector3d> Vect3;
typedef std::vector<std::time_t> Timevect;

struct utils
{
    static Eigen::Matrix3d get_rotmat(const Eigen::Vector3d& angles);
    static Eigen::Vector3d get_wie(const double Lat, const double h);
    static Eigen::Vector3d get_wen(const double Lat, const Eigen::Vector3d& v, const double h);
    static Eigen::Vector3d get_local_gravity(const double Lat, const double h, const Eigen::Vector3d& wie);
    static std::vector<double> compute_RmRn(const double Lat);
    double rmse(Vect3 v1, Vect3 v2);
}