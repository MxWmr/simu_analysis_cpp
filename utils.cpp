#include "utils.hpp"


typedef std::vector<Eigen::Vector3d> Vect3;
typedef std::vector<std::time_t> Timevect;

Eigen::Matrix3d utils::rotationMatrix(const Eigen::Vector3d& angles)
{
    double phi,theta,psi = angles.x(), angles.y(), angles.z();

	double c_phi = std::cos(phi);
	double s_phi = std::sin(phi);

	double c_theta = std::cos(theta);
	double s_theta = std::sin(theta);

	double c_psi = std::cos(psi);
	double s_psi = std::sin(psi);

	Eigen::Matrix3d rotation;

	rotation << c_theta * c_psi, c_psi* s_theta* s_phi - c_phi * s_psi, c_phi* c_psi* s_theta + s_phi * s_psi,
		c_theta* s_psi, s_phi* s_theta* s_psi + c_phi * c_psi, c_phi* s_theta* s_psi - c_psi * s_phi,
		-s_theta, c_theta* s_phi, c_phi* c_theta;

	return rotation;
}


Eigen::Vector3d utils::get_wie(const double Lat, const double h)
{
    double w = 7.292115*1e-5; 
    Eigen::Vector3d wie;
    wie[0] = 0.;
    wie[1] = w*std::cos(Lat);
    wie[2] = w*std::sin(Lat);
    return wie;
}


Eigen::Vector3d utils::get_wen(const double Lat, const Eigen::Vector3d& v, const double h);
{
    Eigen::Vector3d wen;
    std::vector<double> RmRn = compute_RmRn(Lat);
    Rm,Rn = RmRn[0],RmRn[1];

    wen[0] = -v[1]/(Rm+h);
    wen[1] = v[0]/(Rn+h);
    wen[2] = v[0]*std::tan(Lat)/(Rn+h);
    return wen;
}

Eigen::Vector3d utils:: get_local_gravity(const double Lat, const double h, const Eigen::Vector3d& wie)
{
    Eigen::Vector3d gl;
    double g0 = 9.780326;
    double g = g0*(1+5.27904*1e-3*(std::sin(Lat)**2)+2.32718*1e-5*(std::sin(Lat)**4))-3.085*1e-6*h;
    
    gl[0] = 0.;
    gl[1] = 0.;
    gl[2] = -g;


    double e = 0.08181919;
    std::vector<double> RmRn = compute_RmRn(Lat);
    Rm,Rn = RmRn[0],RmRn[1];

    Eigen::Vector3d r;
    r[0] = 0.;
    r[1] = 0.;
    r[2] = (1-e**2)*Rn+h;   // sure ?

    gl = gl - wie.cross(wie.cross(r))      
    return gl;
}

static std::vector<double> utils::compute_RmRn(const double Lat){

    double R = 6378137.0;
    double e = 0.08181919;
    std::vector<double> RmRn(2);
    RmRn[0] = R*(1-e**2)/(1-e**2*mt.sin(Lat)**2)**(3/2)
    RmRn[1] = R/(1-e**2*mt.sin(Lat)**2)**(1/2)

    return RmRn;
}

static double utils::rmse(Vect3 v1, Vect3 v2){
    double rmse = 0;
    for (int i=0,i<v1.size(),i++)[
        rmse+=(v1[i]-v2[i])**2;
    ]
    rmse /= v1.size();
    rmse = std::sqrt(rmse);
    return rmse;
}


    