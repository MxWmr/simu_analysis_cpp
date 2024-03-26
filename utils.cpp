#include "utils.hpp"
#include <cmath>
#include <iostream>
#include <fstream>


Eigen::Vector3d utils::get_wie(const double Lat, const double h)
{
    double w = 7.29211505e-5; 
    Eigen::Vector3d wie;
    wie[0] = 0.;
    wie[1] = w*std::cos(Lat);
    wie[2] = w*std::sin(Lat);
    return wie;
}


Eigen::Vector3d utils::get_wen(const double Lat, const Eigen::Vector3d& v, const double h)
{
    Eigen::Vector3d wen;
    std::vector<double> RmRn = compute_RmRn(Lat);
    double Rm = RmRn[0];
    double Rn = RmRn[1];

    wen[0] = -v[1]/(Rm+h);
    wen[1] = v[0]/(Rn+h);
    wen[2] = v[0]*std::tan(Lat)/(Rn+h);
    return wen;
}


Eigen::Vector3d utils:: get_g(const double Lat, const double h)
{
    Eigen::Vector3d g;
    double g0 = 9.780327;
    double g_ = g0*(1+5.2790414e-3*(std::pow(std::sin(Lat),2))+2.32718e-5*(std::pow(std::sin(Lat),4))+1.262e-7*(std::pow(std::sin(Lat),6))+7e-10*(std::pow(std::sin(Lat),8)))-3.085e-6*h;
 
    g[0] = 0.;
    g[1] = 0.;
    g[2] = -g_;

    return g;
}


Eigen::Vector3d utils:: get_local_gravity(const double Lat, const double h, const Eigen::Vector3d& wie)
{
    Eigen::Vector3d gl = get_g(Lat,h);

    double f =  1 / 298.257223563;
    double e2 = f * (2 - f);
    std::vector<double> RmRn = compute_RmRn(Lat);
    double Rm = RmRn[0];
    double Rn = RmRn[1];

    Eigen::Vector3d r;
    r[0] = 0.;
    r[1] = 0.;
    r[2] = (1-e2)*Rn+h;   

    gl = gl - wie.cross(wie.cross(r));
    return gl;
}

std::vector<double> utils::compute_RmRn(const double Lat)
{

    double R = 6378137.0;
    double f =  1 / 298.257223563;
    double e2 = f * (2 - f);
    std::vector<double> RmRn(2);
    RmRn[0] = R*(1-e2)/std::pow((1-e2*std::pow(std::sin(Lat),2)),3./2.);
    RmRn[1] = R/std::sqrt(1-e2*std::pow(std::sin(Lat),2));

    return RmRn;
}

double utils::rmse(std::vector<Eigen::Vector3d> v1, std::vector<Eigen::Vector3d> v2){
    double rmse = 0;
    for (int i=0;i<v1.size();i++){
        rmse+= (v1[i]-v2[i]).array().square().mean();
    }
    rmse /= v1.size();
    rmse = std::sqrt(rmse);
    return rmse;
}



// fct from:
// https://github.com/AleksandarHaber/Save-and-Load-Eigen-Cpp-Matrices-Arrays-to-and-from-CSV-files/blob/master/source_file.cpp#L22
Eigen::MatrixXd utils::openData(std::string fileToOpen)
{

	// the inspiration for creating this function was drawn from here (I did NOT copy and paste the code)
	// https://stackoverflow.com/questions/34247057/how-to-read-csv-file-and-assign-to-eigen-matrix
	
	// the input is the file: "fileToOpen.csv":
	// a,b,c
	// d,e,f
	// This function converts input file data into the Eigen matrix format



	// the matrix entries are stored in this variable row-wise. For example if we have the matrix:
	// M=[a b c 
	//	  d e f]
	// the entries are stored as matrixEntries=[a,b,c,d,e,f], that is the variable "matrixEntries" is a row vector
	// later on, this vector is mapped into the Eigen matrix format
	std::vector<double> matrixEntries;

	// in this object we store the data from the matrix
	std::ifstream matrixDataFile(fileToOpen);

	// this variable is used to store the row of the matrix that contains commas 
	std::string matrixRowString;

    // skip first line
    getline(matrixDataFile, matrixRowString);

	// this variable is used to store the matrix entry;
	std::string matrixEntry;

	// this variable is used to track the number of rows
	int matrixRowNumber = 0;


	while (std::getline(matrixDataFile, matrixRowString)) // here we read a row by row of matrixDataFile and store every line into the string variable matrixRowString
	{
		std::stringstream matrixRowStringStream(matrixRowString); //convert matrixRowString that is a string to a stream variable.
        

		while (std::getline(matrixRowStringStream, matrixEntry, ',')) // here we read pieces of the stream matrixRowStringStream until every comma, and store the resulting character into the matrixEntry
		{
			matrixEntries.push_back(std::stod(matrixEntry));   //here we convert the string to double and fill in the row vector storing all the matrix entries
		}
		matrixRowNumber++; //update the column numbers
	}

	// here we convet the vector variable into the matrix and return the resulting object, 
	// note that matrixEntries.data() is the pointer to the first memory location at which the entries of the vector matrixEntries are stored;
	return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(matrixEntries.data(), matrixRowNumber, matrixEntries.size() / matrixRowNumber);

}



std::vector<Eigen::Vector3d> utils::mat2vec3d(Eigen::MatrixXd mat){
    std::vector<Eigen::Vector3d> out;
    for (int i=0;i<mat.rows();i++){
        out.push_back(Eigen::Vector3d{mat(i,Eigen::all)});
    }
    return out;
}

void utils::process_orientation(std::vector<Eigen::Vector3d>& orientation){

    for (int i(0); i < orientation.size(); i++)
	{
        std::swap(orientation[i].x(),orientation[i].z());
        std::swap(orientation[i].x(),orientation[i].y());
		//Heading to Yaw
		orientation[i].z() = 90 - orientation[i].z();
	}
}





 std::vector<Eigen::Vector3d> utils::interpolateAngles3d_withRate(const std::vector<Eigen::Vector3d>& input, const double& rate, const double& indexOffset, const bool& radiant)
{
    if (rate == 1.) return std::vector<Eigen::Vector3d>(input); // If rate = 1, there is no interpolation


    std::size_t outputSize = std::size_t(std::floor(input.size() / rate));
    std::vector<Eigen::Vector3d> output(outputSize);

    double inputIndex = indexOffset;
    std::size_t outputIndex = 0;

    double period;
    if (radiant) period = 2 * EIGEN_PI;
    else period = 360;

    while (std::size_t(inputIndex) < input.size() && outputIndex < output.size())
    {
        Eigen::Vector3d previous = input[std::size_t(std::floor(inputIndex))];
        Eigen::Vector3d next = input[std::size_t(std::floor(inputIndex)) + 1];

        double interpolation_coefficent = inputIndex - std::size_t(std::floor(inputIndex));

        Eigen::Vector3d deltaAngles = fmodVector(next - previous, period);
        Eigen::Vector3d shortestDistance = fmodVector(2 * deltaAngles, period) - deltaAngles;
        // std::cout<< <<std::endl;
        // If inputIndex is an integer, interpolation_coefficent = 0 and output[outputIndex] = previous;
        output[outputIndex] = fmodVector(previous + interpolation_coefficent * shortestDistance, period);

        // Increment
        inputIndex += rate;
        outputIndex++;
        
    }


    return output;
};

    std::vector<Eigen::Vector3d> utils::interpolateAngles3d_withRef(const std::vector<Eigen::Vector3d>& input, const std::vector<double>& inputTime, const std::vector<double>& refTime, const bool& radiant) 
{
    std::vector<Eigen::Vector3d> output;
    
    double period;
    if (radiant) period = 2 * PI;
    else period = 360;

    int inputIndex(0);

    for (int refIndex(0); refIndex < refTime.size(); refIndex++)
    {
        std::time_t currentTime = refTime[refIndex];

        while (inputIndex < inputTime.size() && inputTime[inputIndex] < currentTime)
        {
            inputIndex++;
        }

        if (inputIndex == inputTime.size())
        {
            break;
        }
        else if (inputTime[inputIndex] == currentTime)
        {
            output.push_back(input[inputIndex]);
        }
        else
        {
            std::time_t previousTime = inputTime[inputIndex - 1];
            std::time_t nextTime = inputTime[inputIndex];
            double alpha = (currentTime - previousTime) / double(nextTime - previousTime);

            //Angle Process for interpolation
            Eigen::Vector3d deltaAngle = fmodVector(input[inputIndex] - input[inputIndex - 1], period);
            Eigen::Vector3d shortestDistance = fmodVector(2 * deltaAngle, period) - deltaAngle;
            output.push_back(fmodVector(input[inputIndex - 1] + alpha * shortestDistance, period));
        }
    }

    //If interpolation didn't update the last elements
    while (output.size() < refTime.size())
    {
        output.push_back(output.back());
    }

    return output;
}

 std::vector<double> utils::interpolateAngles(const std::vector<double>& input, const double& rate, const double& indexOffset, const bool& radiant)
{
    if (rate == 1.) return std::vector<double>(input); // If rate = 1, there is no interpolation


    std::size_t outputSize = std::size_t(std::floor(input.size() / rate));
    std::vector<double> output(outputSize);

    double inputIndex = indexOffset;
    std::size_t outputIndex = 0;

    double period;
    if (radiant) period = 2 * EIGEN_PI;
    else period = 360;

    while (std::size_t(inputIndex) < input.size() && outputIndex < output.size())
    {
        double previous = input[std::size_t(std::floor(inputIndex))];
        double next = input[std::size_t(std::floor(inputIndex)) + 1];

        double interpolation_coefficent = inputIndex - std::size_t(std::floor(inputIndex));

        double deltaAngles = fmod(next - previous, period);
        double shortestDistance = fmod(2 * deltaAngles, period) - deltaAngles;
        // std::cout<< <<std::endl;
        // If inputIndex is an integer, interpolation_coefficent = 0 and output[outputIndex] = previous;
        output[outputIndex] = fmod(previous + interpolation_coefficent * shortestDistance, period);

        // Increment
        inputIndex += rate;
        outputIndex++;
    }
    return output;
};


Eigen::Vector3d utils::fmodVector(const Eigen::Vector3d& input,const double num){
    Eigen::Vector3d output;
    for (int i=0;i<input.size();i++){
        output[i] = fmod(input[i],num);
    }
    return output;
};


std::vector<Eigen::Vector3d> utils::geo2enu(const std::vector<Eigen::Vector3d>& geoPositions)
{
	std::vector<Eigen::Vector3d> enuPositions(geoPositions.size());

	enuPositions[0] = Eigen::Vector3d::Zero();

	//Get ENU Speed with Geodetic Position variations
	double lat;
	double alt;

	double RN;
	double RM;

	for (std::size_t i = 1; i < enuPositions.size(); i++)
	{
		lat = geoPositions[i].x();
		alt = geoPositions[i].z();

		RN = primeVerticalRadius(lat);
		RM = meridionalRadius(lat);

		enuPositions[i].x() = enuPositions[i - 1].x() + (RN + alt) * (geoPositions[i].y() - geoPositions[i - 1].y()) * std::cos(lat);
		enuPositions[i].y() = enuPositions[i - 1].y() + (RM + alt) * (geoPositions[i].x() - geoPositions[i - 1].x());
		enuPositions[i].z() = enuPositions[i - 1].z() + (geoPositions[i].z() - geoPositions[i - 1].z());
	}

	return enuPositions;
};

double utils::primeVerticalRadius(const double& latitude) 
{
	double s_lat = std::sin(latitude);
    double a = 6378137.0;
    double f = 1 / 298.257223563;
    double e2 = f * (2 - f);
	return a / std::sqrt(1 - e2 * s_lat * s_lat);
};

double utils::meridionalRadius(const double& latitude) 
{
	double RN = primeVerticalRadius(latitude);
    double a = 6378137.0;
    double f = 1 / 298.257223563;
    double e2 = f * (2 - f);
	return std::pow(RN, 3) * (1 - e2) / (a * a);
};