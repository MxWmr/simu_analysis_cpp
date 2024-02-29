#include "imu_process.hpp"
#include <vector>
#include <thread>
#include <ceres/ceres.h>
#include "utils.hpp"
#include "DataWriter.hpp"

// typedef std::vector<Eigen::Vector3d> Vect3;
// typedef std::vector<double> Timevect;


imu_process::imu_process(std::string path, std::string simu_name){
    m_simu_name = simu_name;
    m_path = path;
}

void imu_process::get_data(){

    std::cout << "Loading data ..."<<std::endl;

    std::string full_path = m_path + m_simu_name;

    // TO FINISH
    
    // DVL
    Eigen::MatrixXd dvlmat = utils::openData(full_path+"DVL.csv");
    m_dvltime = utils::mat2vec<double>(dvlmat(Eigen::all,0));
    m_dvlspeed = utils::mat2vec3d(dvlmat(Eigen::all,Eigen::seq(1,3)));

    // USBL
    Eigen::MatrixXd usblmat = utils::openData(full_path+"USBL.csv");
    m_usbltime = utils::mat2vec<double>(usblmat(Eigen::all,0));
    m_usbllat = utils::mat2vec<double>(usblmat(Eigen::all,1));
    // m_usblcoord = usblmat(Eigen::all,Eigen::seq(1,3));
    
    // IMU
    Eigen::MatrixXd imumat = utils::openData(full_path+"IMU.csv");
    m_imutime = utils::mat2vec<double>(imumat(Eigen::all,0));
    m_imuaccel = utils::mat2vec3d(imumat(Eigen::all,Eigen::seq(4,6)));    

    // DEPTH
    Eigen::MatrixXd depthmat = utils::openData(full_path+"Depth.csv");
    m_depthtime = utils::mat2vec<double>(depthmat(Eigen::all,0));
    m_depth = utils::mat2vec<double>(depthmat(Eigen::all,1));    
    m_depth_dvltime = utils::interpolate<double>(m_depth,m_depthtime,m_dvltime);

    //Ref
    Eigen::MatrixXd refmat = utils::openData(full_path+"Reference.csv");
    m_reftime = utils::mat2vec<double>(refmat(Eigen::all,0));
    m_refspeed = utils::mat2vec3d(refmat(Eigen::all,Eigen::seq(4,6)));
    m_refpos = utils::mat2vec3d(refmat(Eigen::all,Eigen::seq(1,3)));  

    m_refpos_dvltime = utils::interpolateVector(m_refpos,m_reftime,m_dvltime);
    m_refpos_imutime = utils::interpolateVector(m_refpos,m_reftime,m_imutime);

    m_refspeed_dvltime = utils::interpolateVector(m_refspeed,m_reftime,m_dvltime);
    m_refspeed_imutime = utils::interpolateVector(m_refspeed,m_reftime,m_imutime);

    // m_refspeed_dvltime = utils::deriveVector(m_refpos_dvltime);
    // m_refspeed_imutime = utils::deriveVector(m_refpos_imutime);

    Eigen::MatrixXd ormat = utils::openData(full_path+"Orientation.csv");
    m_ortime = utils::mat2vec<double>(ormat(Eigen::all,0));
    m_orientation = utils::mat2vec3d(refmat(Eigen::all,Eigen::seq(1,3)));  
    utils::process_orientation(m_orientation);
    m_orientation_dvltime = utils::interpolateVector(m_orientation,m_ortime,m_dvltime);
    m_orientation_imutime = utils::interpolateVector(m_orientation,m_ortime,m_imutime); 

    std::cout << "Done !"<<std::endl;


}

void imu_process::orient_dvl(){
    std::cout << "Orient DVL ..."<<std::endl;

    for (int i=0;i<m_dvlspeed.size();i++){
        Eigen::Matrix3d orient_mat = utils::get_rotmat(m_orientation_dvltime[i]);
        m_dvlspeed[i] = m_misalignement * (orient_mat * m_dvlspeed[i]);
    }

    std::cout << "Done !" << std::endl;
}

void imu_process::orient_imu(){
    std::cout << "Orient IMU ..."<<std::endl;
    int k(0),l(0);
    for (int i=0;i<m_imuaccel.size();i++){
        Eigen::Vector3d v = m_dvlspeed[k];
        double h = m_depth_dvltime[k];
        double Lat = m_usbllat[l];
        Eigen::Matrix3d orient_mat = utils::get_rotmat(m_orientation_imutime[i]);
        // TO COMPLETE
        Eigen::Vector3d wie = utils::get_wie(Lat,h);
        Eigen::Vector3d wen = utils::get_wen(Lat,v,h);
        m_imuaccel[i] = orient_mat * m_imuaccel[i]*50. - utils::get_local_gravity(Lat,h,wie) - (2*wie+wen).cross(v);
        if (m_dvltime[k]>m_imutime[i-1] && m_dvltime[k]<m_imutime[i] && k<m_dvltime.size()-1){
            k+=1;
        }               
        if (m_usbltime[l]>m_imutime[i-1] and m_usbltime[l]<m_imutime[i] and l<m_usbltime.size()-1){
            l+=1;
        }

    }
    std::cout << "Done !"<<std::endl;
}


void imu_process::integrate_dvl(){
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


void imu_process::integrate_imu1(){
    std::cout << "Integrate IMU ..."<<std::endl;

    Eigen::Vector3d v_0 = m_initspeed;
    std::vector<Eigen::Vector3d>  imuspeed;
    imuspeed.push_back(v_0);

    for (int i=1;i<m_imuaccel.size();i++){
        imuspeed.push_back(imuspeed[i-1] + (m_imutime[i]-m_imutime[i-1])*(m_imuaccel[i]+m_imuaccel[i-1])*0.5); 
    }

    m_imuspeed = imuspeed;
    std::cout << "Done !"<<std::endl;
}


void imu_process::integrate_imu2(){
    std::cout << "Integrate IMU ..."<<std::endl;

    Eigen::Vector3d p_0 = m_initpos;
    std::vector<Eigen::Vector3d>  imupos;
    imupos.push_back(p_0);

    for (int i=1;i<m_imuaccel.size();i++){
        imupos.push_back(imupos[i-1] + (m_imutime[i]-m_imutime[i-1])*(m_imuspeed[i]+m_imuspeed[i-1])*0.5); 
    }

    m_imupos = imupos;
    std::cout << "Done !"<<std::endl;
}


std::vector<double>  imu_process::imu_rmse(){
    std::vector<double> res(2);
    
    res[0]=utils::rmse(m_imuspeed,m_refspeed_imutime);
    res[1]=utils::rmse(m_imupos,m_refpos_imutime);

    return res;

}

std::vector<double>  imu_process::dvl_rmse(){
    std::vector<double> res(2);
    
    res[0]=utils::rmse(m_dvlspeed,m_refspeed_dvltime);
    res[1]=utils::rmse(m_dvlpos,m_refpos_dvltime);

    return res;

}

void imu_process::export_results(){

	std::cout << "Starting export ... " << std::endl;

    {
	DataWriter writer(10, "CSV");
    writer.setFolderPath("/home/maxwmr/Documents/work/simu_analysis_cpp/results");

	writer.addData(m_imutime, "time");
	writer.addData(m_imuaccel, "acc1", "acc2", "acc3");
    writer.addData(m_imuspeed, "speedX", "speedY", "speedZ");
    writer.addData(m_refspeed_imutime, "speedX_ref", "speedY_ref", "speedZ_ref");
    writer.addData(m_imupos, "posX", "posY", "posZ");
    writer.addData(m_refpos_imutime, "posX_ref", "posY_ref", "posZ_ref");

	writer.createFile("IMU.csv");
    }

    {
	DataWriter writer(10, "CSV");
    writer.setFolderPath("/home/maxwmr/Documents/work/simu_analysis_cpp/results");

	writer.addData(m_dvltime, "time");
    writer.addData(m_dvlspeed, "speedX", "speedY", "speedZ");
    writer.addData(m_refspeed_dvltime, "speedX_ref", "speedY_ref", "speedZ_ref");
    writer.addData(m_dvlpos, "posX", "posY", "posZ");
    writer.addData(m_refpos_dvltime, "posX_ref", "posY_ref", "posZ_ref");

	writer.createFile("DVL.csv");
    }
    
    std::cout << "Done !"<<std::endl;


}