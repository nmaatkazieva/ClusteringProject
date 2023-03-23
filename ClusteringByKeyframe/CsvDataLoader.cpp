///Std Includes///
#include <fstream>
#include <sstream>

///Clustering includes///
#include "CsvDataLoader.hpp"


using namespace Clustering;

/////////////////////////////////////////////
//
/////////////////////////////////////////////
CsvDataLoader::CsvDataLoader(std::string filePath)
{
	p_filePath = filePath;		//init the file path
}

/////////////////////////////////////////////
//
/////////////////////////////////////////////
void CsvDataLoader::Process() noexcept
{
	std::map<std::string, std::vector<float>> csvData = ReadCsvData();		//load map of point cloud from csv
	FillPointCloud(csvData);												//fill point cloud of Vertex2D
}

/////////////////////////////////////////////
//
/////////////////////////////////////////////
std::vector<Vertex2D> CsvDataLoader::GetData() noexcept
{
	return m_pointCloud;							//return point cloud
}

/////////////////////////////////////////////
//
/////////////////////////////////////////////
std::map<std::string, std::vector<float>> CsvDataLoader::ReadCsvData() noexcept
{
	std::map<std::string, std::vector<float>> data{};		//data to return
	//load csv file 
	std::ifstream file{ p_filePath };
	//check if file loaded
	if (file.is_open())//if file is loaded
	{
		std::string line{}; // line of csv 
		//get header 
		std::getline(file, line);
		std::stringstream stream(line); // set header as stream
		std::string key1{};				// 1st column header
		std::string key2{};				// 2nd column header
		//get 1st column header
		std::getline(stream, key1, ',');
		//get 2nd column header
		std::getline(stream, key2, ',');
		//read all lines of csv
		while (std::getline(file, line))//while line exists
		{
			//read line as stream
			std::stringstream strStream(line);
			char comma{};				// comma char that separates columns
			double lat{};				//latitude value
			double lon{};				//longitude value
			//separate line by comma  and get values
			strStream >> lat >> comma >> lon;
			//fill vector of latitude values
			data[key1].push_back(lat);
			//fill vector of longitude values
			data[key2].push_back(lon);
		}//end while
	}//end if
	else//if file is not loaded
	{
		std::cout << "Could not load CSV file! " << std::endl;
	}//end else

	return data;		//return data
}

/////////////////////////////////////////////
//
/////////////////////////////////////////////
void  CsvDataLoader::FillPointCloud(std::map<std::string, std::vector<float>> csvData) noexcept
{
	//go through all way points and fill waypoints cloud
	for (int index = 0; index < csvData["x"].size(); index++)
	{
		Vertex2D point{};					//create point
		point.x = csvData["x"][index];		//set point's X
		point.y = csvData["y"][index];		//set point's Y
		m_pointCloud.push_back(point);		//add point to point cloud
	}
}
