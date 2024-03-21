#include "imu_process.hpp"


imu_process::imu_process(std::string path, std::string simu_name){
    m_simu_name = simu_name;
    m_path = path;
}

void imu_process::get_data(const bool simu){
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
    m_orientation_b2n = utils::mat2vec3d(ormat(Eigen::all,Eigen::seq(1,3)));  
    utils::process_orientation(m_orientation_b2n);

    // IMU
    Eigen::MatrixXd imumat = utils::openData(full_path+"IMU.csv");
    m_imutime = utils::mat2vec<double>(imumat(Eigen::all,0));
    m_imuaccel_b = utils::mat2vec3d(imumat(Eigen::all,Eigen::seq(4,6)));    

    // DVL
    Eigen::MatrixXd dvlmat = utils::openData(full_path+"DVL.csv");
    m_dvltime = utils::mat2vec<double>(dvlmat(Eigen::all,0));
    m_dvlspeed_b = utils::mat2vec3d(dvlmat(Eigen::all,Eigen::seq(1,3)));

    // Depth
    Eigen::MatrixXd depthmat = utils::openData(full_path+"Depth.csv");
    m_depthtime = utils::mat2vec<double>(depthmat(Eigen::all,0));
    m_depth = utils::mat2vec<double>(depthmat(Eigen::all,1));    
    for (int i(0);i<m_depth.size();i++){m_depth[i]*=-1;}


    //PHINS
    Eigen::MatrixXd phinsmat = utils::openData(full_path+"PHINS.csv");
    m_phinstime = utils::mat2vec<double>(phinsmat(Eigen::all,0));
    m_phinspos = utils::mat2vec3d(phinsmat(Eigen::all,Eigen::seq(1,3)));
    m_phinspos = utils::geo2enu(m_phinspos);  
    m_phinslat =  utils::mat2vec<double>(phinsmat(Eigen::all,1));

    if (!simu)
    {
        // Keep only data in same intervall 
        keepDataInSameInterval();
    }



    // m_orientation_b2n_imutime = m_orientation_b2n;
    m_orientation_b2n_dvltime = utils::interpolateAngles3d(m_orientation_b2n,10,0);

    // let imu data in orientation time (10Hz)
    // m_imuaccel = utils::interpolateVector(m_imuaccel,m_imutime,m_ortime);
    // m_imutime = m_ortime;

    // let imu data remain in imutime (50Hz)
    m_orientation_b2n_imutime = utils::interpolateAngles3d(m_orientation_b2n,0.2,0);
    m_ortime = m_imutime;

    // apply coefficients to have acceleration in the good units
    if (simu)
    {
        for (int i(0);i<m_imutime.size();i++){
            m_imuaccel_b[i] *= 50.   ;
        }
    }
    else
    {
        for (int i(0);i<m_imutime.size();i++){
            m_imuaccel_b[i] *= 50.*1e-7    ;
        }
    }


    m_depth_imutime = utils::interpolate<double>(m_depth,m_depthtime,m_imutime);

    m_phinslat_imutime = utils::interpolateAngles(m_phinslat,0.2,0);
    m_phinspos_dvltime = utils::interpolateVector(m_phinspos,m_phinstime,m_dvltime);
    m_phinspos_imutime = utils::interpolateVector(m_phinspos,m_phinstime,m_imutime);

    // Parameters
    Eigen::MatrixXd parmat = utils::openData(full_path+"Parameters.csv");
    Eigen::Vector3d angles = Eigen::Vector3d{parmat(Eigen::all,0)};
    m_misalignement_dvl2b = utils::get_rotmat(angles);

    if (simu)
    {
        //Ref

        Eigen::MatrixXd refmat = utils::openData(full_path+"Reference.csv");
        m_reftime = utils::mat2vec<double>(refmat(Eigen::all,0));
        m_refspeed_n = utils::mat2vec3d(refmat(Eigen::all,Eigen::seq(7,9)));
        m_refpos = utils::mat2vec3d(refmat(Eigen::all,Eigen::seq(4,6)));  

        m_refpos_dvltime = utils::interpolateVector(m_refpos,m_reftime,m_dvltime);
        m_refpos_imutime = utils::interpolateVector(m_refpos,m_reftime,m_imutime);

        m_refspeed_n_dvltime = utils::interpolateVector(m_refspeed_n,m_reftime,m_dvltime);
        m_refspeed_n_imutime = utils::interpolateVector(m_refspeed_n,m_reftime,m_imutime);

        m_reflat =  utils::mat2vec<double>(refmat(Eigen::all,1));
        m_refdepth = utils::mat2vec<double>(refmat(Eigen::all,3));
        m_refdepth_imutime = utils::interpolate<double>(m_refdepth,m_reftime,m_imutime);
        m_reflat_imutime = utils::interpolateAngles(m_reflat,5,0);

        m_initpos = m_refpos[0];

    }
    else
    {
        m_refpos_imutime = m_phinspos_imutime;
        m_refdepth_imutime = m_depth_imutime;
        m_reflat_imutime = m_phinslat_imutime;
        m_initpos = m_phinspos[0];

    }






    std::cout << "Done !"<<std::endl;


}


void imu_process::keepDataInSameInterval()
{
	std::vector<double> beginTimes(6);
	beginTimes = { m_dvltime.front(), m_usbltime.front(), m_imutime.front(), m_depthtime.front(), m_ortime.front(), m_phinstime.front()};
	double beginTime = *std::max_element(beginTimes.begin(), beginTimes.end());

	std::vector<double> endTimes(6);
	endTimes = { m_dvltime.back(), m_usbltime.back(), m_imutime.back(), m_depthtime.back(), m_ortime.back(), m_phinstime.back()};
	double endTime = *std::min_element(endTimes.begin(), endTimes.end());

	std::pair<int, int> interval;

	interval = cutTime(m_dvltime, beginTime, endTime);
	cutData<Eigen::Vector3d>(m_dvlspeed_b, interval);

	interval = cutTime(m_usbltime, beginTime, endTime);
	cutData<Eigen::Vector3d>(m_usblgeo, interval);
	cutData<double>(m_usbllat, interval);

	interval = cutTime(m_imutime, beginTime, endTime);
	cutData<Eigen::Vector3d>(m_imuaccel_b, interval);

	interval = cutTime(m_depthtime, beginTime, endTime);
	cutData<double>(m_depth, interval);

	interval = cutTime(m_ortime, beginTime, endTime);
	cutData<Eigen::Vector3d>(m_orientation_b2n, interval);

	interval = cutTime(m_phinstime, beginTime, endTime);
	cutData<Eigen::Vector3d>(m_phinspos, interval);
	cutData<double>(m_phinslat, interval);

}


std::pair<int, int> imu_process::cutTime(std::vector<double>& time, const double& beginTime, const double& endTime)
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





void imu_process::orient_dvl(const bool simu)
{
    std::cout << "Orient DVL ..."<<std::endl;
    m_dvlspeed_n = m_dvlspeed_b;

    for (int i=0;i<m_dvltime.size();i++)
    {
        Eigen::Matrix3d R_b2n = utils::get_rotmat(m_orientation_b2n_dvltime[i]);
        m_dvlspeed_n[i] = R_b2n*(m_misalignement_dvl2b * m_dvlspeed_b[i]);
    }


    // interpolations to imutime
    m_dvlspeed_n_imutime = utils::interpolateVector(m_dvlspeed_n,m_dvltime,m_imutime);

    if (simu){
        m_initspeed = m_refspeed_n[0];
        }
    else{
        m_initspeed = m_dvlspeed_n[0];
        m_refspeed_n_imutime = m_dvlspeed_n_imutime;
        }    
    


    std::cout << "Done !" << std::endl;
}

void imu_process::orient_imu(const bool inert,const Eigen::Vector3d& bias, const double scale_factor)
{
    std::cout << "Orient IMU ..."<<std::endl;
    m_imuaccel_n = m_imuaccel_b;

    for (int i=0;i<m_imutime.size();i++)
    {

        Eigen::Vector3d v = m_refspeed_n_imutime[i];
        double h = m_refdepth_imutime[i]; 
        double Lat = m_reflat_imutime[i]*EIGEN_PI/180.;
        Eigen::Matrix3d R_b2n = utils::get_rotmat(m_orientation_b2n_imutime[i]);

        m_imuaccel_n[i] = (m_imuaccel_b[i]-bias)/scale_factor;
        m_imuaccel_n[i] =  R_b2n * m_imuaccel_n[i];

        // Remove gravity and inertial acceleration
        if (inert)
        {
            Eigen::Vector3d wie = utils::get_wie(Lat,h);
            Eigen::Vector3d wen = utils::get_wen(Lat,v,h);

            m_imuaccel_n[i] = m_imuaccel_n[i] + utils::get_local_gravity(Lat,h,wie) - (2*wie+wen).cross(v);  
        }
        else
        {
            m_imuaccel_n[i] =  m_imuaccel_n[i] + utils::get_g(Lat,h);
        }

    }

    std::cout << "Done !"<<std::endl;
}


void imu_process::integrate_dvl()
{
    std::cout << "Integrate DVL ..."<<std::endl;

    Eigen::Vector3d p_0 = m_initpos;
    std::vector<Eigen::Vector3d>  dvlpos;
    dvlpos.push_back(p_0);

    for (int i=1;i<m_dvltime.size();i++){
        dvlpos.push_back(dvlpos[i-1] + (m_dvltime[i]-m_dvltime[i-1])*(m_dvlspeed_n[i]+m_dvlspeed_n[i-1])*0.5); 
    }

    m_dvlpos = dvlpos;
    std::cout << "Done !"<<std::endl;
}


void imu_process::integrate_imu1()
{
    std::cout << "Integrate IMU ..."<<std::endl;

    Eigen::Vector3d v_0 = m_dvlspeed_n[0];
    std::vector<Eigen::Vector3d>  imuspeed_n;
    imuspeed_n.push_back(v_0);

    imuspeed_n.push_back(imuspeed_n[0] + (m_imutime[1]-m_imutime[0]) * m_imuaccel_n[1]);
    for (int i=2;i<m_imutime.size();i++){
        imuspeed_n.push_back(imuspeed_n[i-1] + (m_imutime[i]-m_imutime[i-1])*(m_imuaccel_n[i]+m_imuaccel_n[i-1])*0.5); 
    }

    m_imuspeed_n = imuspeed_n;
    std::cout << "Done !"<<std::endl;
}


void imu_process::integrate_imu2()
{
    std::cout << "Integrate IMU ..."<<std::endl;

    Eigen::Vector3d p_0 = m_phinspos[0];
    std::vector<Eigen::Vector3d> imupos;
    imupos.push_back(p_0);

    for (int i=1;i<m_imutime.size();i++){
        imupos.push_back(imupos[i-1] + (m_imutime[i]-m_imutime[i-1])*(m_imuspeed_n[i]+m_imuspeed_n[i-1])*0.5); 
    }

    m_imupos = imupos;
    std::cout << "Done !"<<std::endl;
}



void imu_process::export_results(const bool simu){

	std::cout << "Starting export ... " << std::endl;

    {
	DataWriter writer(10, "CSV");
    writer.setFolderPath("/home/maxwmr/Documents/work/simu_analysis_cpp/results");

	writer.addData(m_imutime, "time");
	writer.addData(m_imuaccel_n, "acc1", "acc2", "acc3");
    writer.addData(m_imuspeed_n, "speedX", "speedY", "speedZ");
    writer.addData(m_imupos, "posX", "posY", "posZ");
    if (simu){
    writer.addData(m_refspeed_n_imutime, "speedX_ref", "speedY_ref", "speedZ_ref");
    writer.addData(m_refpos_imutime, "posX_ref", "posY_ref", "posZ_ref");
    }
	writer.createFile("IMU.csv");
    }

    {
	DataWriter writer(10, "CSV");
    writer.setFolderPath("/home/maxwmr/Documents/work/simu_analysis_cpp/results");

	writer.addData(m_dvltime, "time");
    writer.addData(m_dvlspeed_n, "speedX", "speedY", "speedZ");
    writer.addData(m_dvlpos, "posX", "posY", "posZ");
    if (simu){
    writer.addData(m_refspeed_n_dvltime, "speedX_ref", "speedY_ref", "speedZ_ref");
    writer.addData(m_refpos_dvltime, "posX_ref", "posY_ref", "posZ_ref");
    }
    else{
    writer.addData(m_phinspos_dvltime,"posX_ref", "posY_ref", "posZ_ref");
    }
	writer.createFile("DVL.csv");
    }
    
    std::cout << "Done !"<<std::endl;


}


std::vector<double>  imu_process::rmse(){
    std::vector<double> res(2);
    
    res[0]=utils::rmse(m_imuspeed_n,m_refspeed_n_imutime);
    res[1]=utils::rmse(m_imupos,m_refpos_imutime);

    return res;

}




std::vector<double> imu_process::get_imutime(){return m_imutime;}
std::vector<double> imu_process::get_dvltime(){return m_dvltime;}
std::vector<double> imu_process::get_depth_imutime(){return m_depth_imutime;}
std::vector<double> imu_process::get_phinslat_imutime(){return m_phinslat_imutime;}
std::vector<Eigen::Vector3d> imu_process::get_orientation_imutime(){return m_orientation_b2n_imutime;};
std::vector<Eigen::Vector3d> imu_process::get_imuaccel_b(){return m_imuaccel_b;}
std::vector<Eigen::Vector3d> imu_process::get_dvlspeed_n(){return m_dvlspeed_n;}
Eigen::Vector3d imu_process::get_initspeed(){return m_initspeed;}
Eigen::Vector3d imu_process::get_bias(){return m_bias;}
double imu_process::get_scale_factor(){return m_scale_factor;}


void imu_process::find_bias(){


    // initalize parameters
    std::vector<double> estimated_bias =  {0.0, 0.00, -0.0};
    double estimated_scale_factor = 1;
    problem.AddParameterBlock(estimated_bias.data(),3);
    problem.AddParameterBlock(&estimated_scale_factor,1);
    
    
    
    // add residuals
    for (int i(1);i<m_dvltime.size();i++)
    {
        problem.AddParameterBlock(estimated_bias[i].data(),3);

        ceres::CostFunction* f = new ceres::AutoDiffCostFunction<Res_bias_sf, 3, 3>(new Res_bias_sf(*this,i));
        problem.AddResidualBlock(f,nullptr,estimated_bias[i].data()); 

        if (i>0)
        {
        ceres::CostFunction* f2 = new ceres::AutoDiffCostFunction<Res_cstbias, 3, 3, 3>(new Res_cstbias(0.001));
        problem.AddResidualBlock(f2,nullptr,estimated_bias[i].data(),estimated_bias[i-1].data()); 
        }
    }

    //Options
	ceres::Solver::Options options;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;   
	options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.function_tolerance = 1e-7;
    // options.max_num_spse_iterations = 5;
    // options.spse_tolerance = 0.1;
    // options.min_trust_region_radius = 1e-45;

	const unsigned int processor_count = std::thread::hardware_concurrency();
	if (processor_count != 0)
	{
		options.num_threads = processor_count;
	}
	options.max_num_iterations = 300;


        options.minimizer_progress_to_stdout = false;

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        

    }

    //Options

    // options.max_num_spse_iterations = 5;
    // options.spse_tolerance = 0.1;
    // options.min_trust_region_radius = 1e-45;



    std::cout << "first bias: "<< estimated_bias[0]<< std::endl;
    std::cout << "bias in the middle: "<< estimated_bias[estimated_bias.size()/2]<< std::endl;
    std::cout << "last bias: "<< estimated_bias[estimated_bias.size()-1]<< std::endl;


    m_scale_factor = estimated_scale_factor;
    m_bias = Eigen::Vector3d(estimated_bias.data());



}
