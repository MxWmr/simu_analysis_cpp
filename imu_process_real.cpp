#include "imu_process_real.hpp"
#include <vector>
#include <thread>
#include "utils.hpp"
#include "DataWriter.hpp"

// typedef std::vector<Eigen::Vector3d> Vect3;
// typedef std::vector<double> Timevect;


imu_process_real::imu_process_real(std::string path, std::string simu_name){
    m_simu_name = simu_name;
    m_path = path;
}

void imu_process_real::get_data(){
    /* 
    Get all dat from csv file where time is always the first column and is in second
    
    All the angles are in deg. The converiso to deg is done in the utils fct
     */

    std::cout << "Loading data ..."<<std::endl;

    std::string full_path = m_path + m_simu_name;

    // USBL
    Eigen::MatrixXd usblmat = utils::openData(full_path+"USBL.csv");
    m_usbltime = utils::mat2vec<double>(usblmat(Eigen::all,0));
    m_usblgeo = utils::mat2vec3d(usblmat(Eigen::all,Eigen::seq(1,3)));
    m_usbllat = utils::mat2vec<double>(usblmat(Eigen::all,1));
    
    // Orientation
    Eigen::MatrixXd ormat = utils::openData(full_path+"Orientation.csv");
    m_ortime = utils::mat2vec<double>(ormat(Eigen::all,0));
    m_orientation = utils::mat2vec3d(ormat(Eigen::all,Eigen::seq(1,3)));  
    utils::process_orientation(m_orientation);

    // IMU
    Eigen::MatrixXd imumat = utils::openData(full_path+"IMU.csv");
    m_imutime = utils::mat2vec<double>(imumat(Eigen::all,0));
    m_imuaccel = utils::mat2vec3d(imumat(Eigen::all,Eigen::seq(4,6)));    


    // DVL
    Eigen::MatrixXd dvlmat = utils::openData(full_path+"DVL.csv");
    m_dvltime = utils::mat2vec<double>(dvlmat(Eigen::all,0));
    m_dvlspeed = utils::mat2vec3d(dvlmat(Eigen::all,Eigen::seq(1,3)));

    // Depth
    Eigen::MatrixXd depthmat = utils::openData(full_path+"Depth.csv");
    m_depthtime = utils::mat2vec<double>(depthmat(Eigen::all,0));
    m_depth = utils::mat2vec<double>(depthmat(Eigen::all,1));    

    //PHINS
    Eigen::MatrixXd phinsmat = utils::openData(full_path+"PHINS.csv");
    m_phinstime = utils::mat2vec<double>(phinsmat(Eigen::all,0));
    m_phinspos = utils::mat2vec3d(phinsmat(Eigen::all,Eigen::seq(1,3)));
    m_phinspos = utils::geo2enu(m_phinspos);  
    m_phinslat =  utils::mat2vec<double>(phinsmat(Eigen::all,1));



    // Keep only data in same intervall 
    keepDataInSameInterval();


    // interpolations
    m_dvlspeed_imutime = utils::interpolateVector(m_dvlspeed,m_dvltime,m_imutime);

    // m_orientation_imutime = m_orientation;
    m_orientation_dvltime = utils::interpolateAngles3d(m_orientation,10,0);

    // m_imuaccel = utils::interpolateVector(m_imuaccel,m_imutime,m_ortime);
    // m_imutime = m_ortime;
    m_orientation_imutime = utils::interpolateAngles3d(m_orientation,0.2,0);
    m_ortime = m_imutime;

    m_depth_imutime = utils::interpolate<double>(m_depth,m_depthtime,m_imutime);

    m_phinslat_imutime = m_phinslat;
    m_phinspos_dvltime = utils::interpolateVector(m_phinspos,m_phinstime,m_dvltime);
    m_initspeed = m_dvlspeed[0];
    m_initpos = m_phinspos[0];  // Warning ! lat long


    // Parameters
    Eigen::MatrixXd parmat = utils::openData(full_path+"Parameters.csv");
    Eigen::Vector3d angles = Eigen::Vector3d{parmat(Eigen::all,0)};
    m_misalignement = utils::get_rotmat(angles);


    std::cout << "Done !"<<std::endl;


}


void imu_process_real::keepDataInSameInterval()
{
	std::vector<double> beginTimes(6);
	beginTimes = { m_dvltime.front(), m_usbltime.front(), m_imutime.front(), m_depthtime.front(), m_ortime.front(), m_phinstime.front()};
	double beginTime = *std::max_element(beginTimes.begin(), beginTimes.end());

	std::vector<double> endTimes(6);
	endTimes = { m_dvltime.back(), m_usbltime.back(), m_imutime.back(), m_depthtime.back(), m_ortime.back(), m_phinstime.back()};
	double endTime = *std::min_element(endTimes.begin(), endTimes.end());

	std::pair<int, int> interval;

	interval = cutTime(m_dvltime, beginTime, endTime);
	cutData<Eigen::Vector3d>(m_dvlspeed, interval);

	interval = cutTime(m_usbltime, beginTime, endTime);
	cutData<Eigen::Vector3d>(m_usblgeo, interval);
	cutData<double>(m_usbllat, interval);

	interval = cutTime(m_imutime, beginTime, endTime);
	cutData<Eigen::Vector3d>(m_imuaccel, interval);

	interval = cutTime(m_depthtime, beginTime, endTime);
	cutData<double>(m_depth, interval);

	interval = cutTime(m_ortime, beginTime, endTime);
	cutData<Eigen::Vector3d>(m_orientation, interval);

	interval = cutTime(m_phinstime, beginTime, endTime);
	cutData<Eigen::Vector3d>(m_phinspos, interval);
	cutData<double>(m_phinslat, interval);

}


std::pair<int, int> imu_process_real::cutTime(std::vector<double>& time, const double& beginTime, const double& endTime)
{
	int begin(0);
	int end(time.size()-1);

	std::vector<double> newTime;

	while (time[begin] < beginTime && begin < end)
	{
		begin++;
	}
	
	while (time[end] > endTime && begin < end)
	{
		end--;
	}

	cutData<double>(time, std::pair<int, int>(begin, end));

	return std::pair(begin, end);
}





void imu_process_real::orient_dvl(){
    std::cout << "Orient DVL ..."<<std::endl;

    for (int i=0;i<m_dvlspeed.size();i++){
        Eigen::Matrix3d R_b2n = utils::get_rotmat(m_orientation_dvltime[i]);
        m_dvlspeed[i] = R_b2n*(m_misalignement * m_dvlspeed[i]);
    }

    std::cout << "Done !" << std::endl;
}

void imu_process_real::orient_imu(const bool inert){
    std::cout << "Orient IMU ..."<<std::endl;
    // int k(0),l(0);
    for (int i=0;i<m_imuaccel.size();i++){
        Eigen::Vector3d v = m_dvlspeed_imutime[i];
        double h = -m_depth_imutime[i]; // - because depth is not altitude !!
        double Lat = m_phinslat_imutime[i]*EIGEN_PI/180.;

        Eigen::Matrix3d R_b2n = utils::get_rotmat(m_orientation_imutime[i]);
        // TO COMPLETE
        if (inert){
            Eigen::Vector3d wie = utils::get_wie(Lat,h);
            Eigen::Vector3d wen = utils::get_wen(Lat,v,h);
            m_imuaccel[i] = R_b2n * m_imuaccel[i]*50.*1e-7 + utils::get_local_gravity(Lat,h,wie) - (2*wie+wen).cross(v);  
        }
        else{
            m_imuaccel[i] =  R_b2n * m_imuaccel[i]*50.*1e-7 + utils::get_g(Lat,h);
            std::cout <<  utils::get_g(Lat,h) << std::endl;
        }


    }

    // std::vector<Eigen::Vector3d> dvlspeed_imutime =  utils::interpolateVector(m_dvlspeed,m_dvltime,m_imutime);
    // for (int i=0;i<dvlspeed_imutime.size();i++){
    //      m_imuaccel[i] = m_imuaccel[i] - dvlspeed_imutime[i];
    // }

    std::cout << "Done !"<<std::endl;
}


void imu_process_real::integrate_dvl(){
    std::cout << "Integrate DVL ..."<<std::endl;

    Eigen::Vector3d p_0 = m_initpos;
    std::vector<Eigen::Vector3d>  dvlpos;
    dvlpos.push_back(p_0);

    for (int i=1;i<m_dvlspeed.size();i++){
        dvlpos.push_back(dvlpos[i-1] + (m_dvltime[i]-m_dvltime[i-1])*(m_dvlspeed[i]+m_dvlspeed[i-1])*0.5); 
    }

    m_dvlpos = dvlpos;
    std::cout << "Done !"<<std::endl;
}


void imu_process_real::integrate_imu1(){
    std::cout << "Integrate IMU ..."<<std::endl;

    Eigen::Vector3d v_0 = m_initspeed;
    std::vector<Eigen::Vector3d>  imuspeed;
    imuspeed.push_back(v_0);
    imuspeed.push_back(imuspeed[0] + (m_imutime[1]-m_imutime[0]) * m_imuaccel[1]);
    for (int i=2;i<m_imuaccel.size();i++){
        imuspeed.push_back(imuspeed[i-1] + (m_imutime[i]-m_imutime[i-1])*(m_imuaccel[i]+m_imuaccel[i-1])*0.5); 
    }

    m_imuspeed = imuspeed;
    std::cout << "Done !"<<std::endl;
}


void imu_process_real::integrate_imu2(){
    std::cout << "Integrate IMU ..."<<std::endl;

    Eigen::Vector3d p_0 = m_initpos;
    std::vector<Eigen::Vector3d> imupos;
    imupos.push_back(p_0);

    for (int i=1;i<m_imuaccel.size();i++){
        imupos.push_back(imupos[i-1] + (m_imutime[i]-m_imutime[i-1])*(m_imuspeed[i]+m_imuspeed[i-1])*0.5); 
    }

    m_imupos = imupos;
    std::cout << "Done !"<<std::endl;
}



void imu_process_real::export_results(){

	std::cout << "Starting export ... " << std::endl;

    {
	DataWriter writer(10, "CSV");
    writer.setFolderPath("/home/maxwmr/Documents/work/simu_analysis_cpp/results");

	writer.addData(m_imutime, "time");
	writer.addData(m_imuaccel, "acc1", "acc2", "acc3");
    writer.addData(m_imuspeed, "speedX", "speedY", "speedZ");
    writer.addData(m_imupos, "posX", "posY", "posZ");

	writer.createFile("IMU.csv");
    }

    {
	DataWriter writer(10, "CSV");
    writer.setFolderPath("/home/maxwmr/Documents/work/simu_analysis_cpp/results");

	writer.addData(m_dvltime, "time");
    writer.addData(m_dvlspeed, "speedX", "speedY", "speedZ");
    writer.addData(m_dvlpos, "posX", "posY", "posZ");
    writer.addData(m_phinspos_dvltime,"posX_ref", "posY_ref", "posZ_ref");
	writer.createFile("DVL.csv");
    }
    
    std::cout << "Done !"<<std::endl;


}