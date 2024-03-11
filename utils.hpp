#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <fstream>
#include <vector>
#include <random>
#include <string>
#include <vector>
#include <iomanip>
#include <algorithm> 
#include <cmath>

#include <Eigen/Dense>


const double PI = 3.141592653589793;

struct utils
{
    static Eigen::Matrix3d get_rotmat(const Eigen::Vector3d& angles);
    static Eigen::Vector3d get_wie(const double Lat, const double h);
    static Eigen::Vector3d get_wen(const double Lat, const Eigen::Vector3d& v, const double h);
    static Eigen::Vector3d get_g(const double Lat, const double h);
    static Eigen::Vector3d get_local_gravity(const double Lat, const double h, const Eigen::Vector3d& wie);
    static std::vector<double> compute_RmRn(const double Lat);
    static double rmse(std::vector<Eigen::Vector3d> v1, std::vector<Eigen::Vector3d> v2);
    static Eigen::MatrixXd openData(std::string fileToOpen);
	static std::vector<Eigen::Vector3d> mat2vec3d(Eigen::MatrixXd mat);
	static void process_orientation(std::vector<Eigen::Vector3d>& orientation);
	static std::vector<Eigen::Vector3d> interpolateAngles3d(const std::vector<Eigen::Vector3d>& input, const double& rate, const double& indexOffset, const bool& radiant = false);
	static std::vector<double> interpolateAngles(const std::vector<double>& input, const double& rate, const double& indexOffset, const bool& radiant = false);
	static Eigen::Vector3d fmodVector(const Eigen::Vector3d& input,const double num);
	static std::vector<Eigen::Vector3d> geo2enu(const std::vector<Eigen::Vector3d>& geoPositions);
	static double primeVerticalRadius(const double& latitude); 
	static double meridionalRadius(const double& latitude); 


	template<typename T>
	static std::vector<T> mat2vec(const Eigen::MatrixXd& mat){
		std::vector<T> out;
		for (int i=0;i<mat.rows();i++)
		{
			out.push_back(T{mat(i,0)});
		}
    	return out;
	}


	template<typename T>
	static std::vector<T> interpolate(const std::vector<double>& input, const std::vector<double>& inputTime, const std::vector<double>& refTime)
	{
		std::vector<T> output;

		std::size_t inputIndex(0);

		for (std::size_t refIndex(0); refIndex < refTime.size(); refIndex++)
		{
			double currentTime = refTime[refIndex];

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
				output.push_back(T(input[inputIndex]));
			}
			else
			{
				double previousTime = inputTime[inputIndex - 1];
				double nextTime = inputTime[inputIndex];
				double alpha = (currentTime - previousTime) / double(nextTime - previousTime);

				output.push_back(T(alpha * input[inputIndex] + (1 - alpha) * input[inputIndex - 1]));
			}
		}

		//If interpolation didn't update the last elements
		while (output.size() < refTime.size())
		{
			output.push_back(output.back());
		}

		return output;
	}


    template<typename T>
	static std::vector<Eigen::Vector3<T>> interpolateVector(const std::vector<Eigen::Vector3<T>>& input, const std::vector<double>& inputTime, const std::vector<double>& refTime)
	{
		std::vector<Eigen::Vector3<T>> output;

		std::size_t inputIndex(0);

		for (std::size_t refIndex(0); refIndex < refTime.size(); refIndex++)
		{
			double currentTime = refTime[refIndex];

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
				double previousTime = inputTime[inputIndex - 1];
				double nextTime = inputTime[inputIndex];
				double alpha = (currentTime - previousTime) / double(nextTime - previousTime);

				output.push_back(alpha * input[inputIndex] + (1 - alpha) * input[inputIndex - 1]);
			}
		}

		//If interpolation didn't update the last elements
		while (output.size() < refTime.size())
		{
			output.push_back(output.back());
		}

		return output;
	}





	// template<typename T>
	// static std::vector<Eigen::Vector3<T>> interpolateAnglesVectors(const std::vector<Eigen::Vector3<T>>  input, const double& rate, const double& indexOffset, const bool& radiant = false)
	// {
	// 	std::vector<T> roll = (,rate,indexOffset,radiant)
	// 	return output;
	// }




};

#endif