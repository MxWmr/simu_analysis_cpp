#include "DataWriter.hpp"

DataWriter::DataWriter(const std::streamsize& digits, const std::string& format)
{
	m_folder_path = "";
	m_digits = digits;
	m_format = format;

	m_csv_format = Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
}

DataWriter::DataWriter(const std::string& format, const std::streamsize& digits)
{
	m_folder_path = "";
	m_digits = digits;
	m_format = format;

	m_csv_format = Eigen::IOFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");
}



void DataWriter::setFolderPath(const std::string& folderPath)
{
	m_folder_path = folderPath;
}

void DataWriter::setPrecision(const std::streamsize& digits)
{
	m_digits = digits;
}

void DataWriter::setFormat(const std::string& format)
{
	m_format = format;
}

std::string DataWriter::getFolderPath() const
{
	return m_folder_path;
}

void DataWriter::createFile(const std::string& fileName, const std::streamsize& digits, const std::string& format)
{
	std::ofstream file;

	//Erase file's content
	file.open(m_folder_path + '/' + fileName, std::ofstream::out | std::ofstream::trunc);
	file.close();

	//Write in the file
	file.open(m_folder_path + '/' + fileName);

	if (file)
	{
		std::cout << "Creating file " << m_folder_path + '/' + fileName << "..." << std::endl;

		file << std::setprecision(digits);

		// CSV Format
		if (format == "CSV")
		{
			// Header - Names
			file << m_names[0];

			for (std::size_t i = 1; i < m_names.size(); i++)
			{
				file << ',' << m_names[i];
			}

			file << '\n';

			// Data
			file << m_data.format(m_csv_format);
		}

		// DELPHINS Format
		else if (format == "DELPHINS")
		{
			/*-------------------------------------------------------------------------------------------------*/
				std::cout << "Can't create file. DELPHINS format has not been implemented yet." << std::endl;
			/*-------------------------------------------------------------------------------------------------*/
		}
	}
	else
	{
		std::cout << "Can't create file. Path might be wrong." << std::endl;
	}

	file.close();
}

void DataWriter::createFile(const std::string& fileName, const std::string& format)
{
	createFile(fileName, m_digits, format);
}

void DataWriter::createFile(const std::string& fileName)
{
	createFile(fileName, m_digits, m_format);
}

void DataWriter::addData(const double& value, const std::string& name)
{
	if (m_data.cols() != 0) // Can't create a vector with a single value without knowing the size of the data
	{
		Eigen::MatrixXd matrix(m_data.rows(), 1);

		for (int i = 0; i < matrix.rows(); i++)
		{
			matrix(i, 0) = value;
		}

		m_names.push_back(name);
		addMatrix(matrix);
	}
	else
	{
		std::cout << "Can't add value, empty matrix data." << std::endl;
	}
}

void DataWriter::addData(const Eigen::Vector3d& value, const std::string& name1, const std::string& name2, const std::string& name3)
{
	if (m_data.cols() != 0) // Can't create a vector with a single value without knowing the size of the data
	{
		Eigen::MatrixXd matrix(m_data.rows(), 3);

		for (int i = 0; i < matrix.rows(); i++)
		{
			matrix(i, 0) = value.x();
			matrix(i, 1) = value.y();
			matrix(i, 2) = value.z();
		}

		m_names.push_back(name1);
		m_names.push_back(name2);
		m_names.push_back(name3);
		addMatrix(matrix);
	}
	else
	{
		std::cout << "Can't add value, empty matrix data." << std::endl;
	}
}

void DataWriter::addData(const std::vector<double>& vector, const std::string& name)
{
	Eigen::MatrixXd matrix(vector.size(), 1);
	for (int i = 0; i < matrix.rows(); i++)
	{
		matrix(i, 0) = vector[i];
	}

	m_names.push_back(name);
	addMatrix(matrix);
}

void DataWriter::addData(const std::vector<Eigen::Vector3d>& vector, const std::string& name1, const std::string& name2, const std::string& name3)
{
	Eigen::MatrixXd matrix(vector.size(), 3);

	for (int i = 0; i < matrix.rows(); i++)
	{
		matrix(i, 0) = vector[i].x();
		matrix(i, 1) = vector[i].y();
		matrix(i, 2) = vector[i].z();
	}

	m_names.push_back(name1);
	m_names.push_back(name2);
	m_names.push_back(name3);
	addMatrix(matrix);
}

void DataWriter::dropData()
{
	m_names.resize(0);
	m_data.resize(0, 0);
}

void DataWriter::addMatrix(const Eigen::MatrixXd& matrix)
{
	if (m_data.cols() == 0) // Empty data matrix, initialization
	{
		m_data.resize(matrix.rows(), matrix.cols());

		m_data << matrix;
	}
	else // Concatenation
	{
		int previousDataCols = m_data.cols();

		m_data.conservativeResize(m_data.rows(), m_data.cols() + matrix.cols());
		
		for (int i = 0; i < matrix.rows(); i++)
		{
			for (int j = 0; j < matrix.cols(); j++)
			{
				m_data(i, previousDataCols + j) = matrix(i, j);
			}
		}
	}
}

