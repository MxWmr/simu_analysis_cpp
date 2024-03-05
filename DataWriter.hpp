#ifndef DATAWRITER_HPP
#define DATAWRITER_HPP

#include <filesystem>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>
#include <Eigen/Dense>

class DataWriter
{
public:
	DataWriter(const std::streamsize& digits = 15, const std::string& format = "CSV");
	DataWriter(const std::string& format, const std::streamsize& digits = 15);

	void setFolderPath(const std::string& folderPath);
	void setPrecision(const std::streamsize& digits);
	void setFormat(const std::string& format);


	std::string getFolderPath() const;


	void createFile(const std::string& fileName, const std::streamsize& digits, const std::string& format);
	void createFile(const std::string& fileName, const std::string& format);
	void createFile(const std::string& fileName);


	void addData(const double& value, const std::string& name);
	void addData(const Eigen::Vector3d& value, const std::string& name1, const std::string& name2, const std::string& name3);

	void addData(const std::vector<double>& vector, const std::string& name);
	void addData(const std::vector<Eigen::Vector3d>& vector, const std::string& name1, const std::string& name2, const std::string& name3);
	
	void dropData();
private:
	void addMatrix(const Eigen::MatrixXd& matrix);

	std::string m_folder_path;
	std::streamsize m_digits;
	std::string m_format;
	
	std::vector<std::string> m_names;
	Eigen::MatrixXd m_data;

	Eigen::IOFormat m_csv_format;
};

#endif
