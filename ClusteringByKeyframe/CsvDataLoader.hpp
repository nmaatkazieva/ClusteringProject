#ifndef CSV_DATA_LOADER_HPP
#define CSV_DATA_LOADER_HPP

///Std Includes///
#include <vector>
#include <iostream>
#include <map>

///Clustering includes///
#include "Vertex2D.h"

namespace Clustering
{
	/// <summary>
	/// Class to load point cloud data from CSV file
	/// </summary>
	class CsvDataLoader final
	{
	public:
		/// <summary>
		/// Standard constructor
		/// </summary>
		/// <param name="filePath">the path of csv file</param>
		CsvDataLoader(std::string filePath);

		/// <summary>
		/// Main func to run all process
		/// </summary>
		/// <returns></returns>
		void Process() noexcept;

		/// <summary>
		/// Func to get point cloud data
		/// </summary>
		/// <returns>point cloud of Vertex2D points</returns>
		std::vector<Vertex2D> GetData() noexcept;

	private:
		/// <summary>
		/// Func read CSV file and load point cloud as map
		/// </summary>
		/// <returns>map of point cloud</returns>
		std::map<std::string, std::vector<float>> ReadCsvData() noexcept;

		/// <summary>
		/// Func to transform map point cloud to Vertex2D point cloud 
		/// </summary>
		/// <param name="csvData">map of point cloud</param>
		/// <returns>filled point cloud member of Vertex2D</returns>
		void FillPointCloud(std::map<std::string, std::vector<float>> csvData) noexcept;

		//////////////////////////
		//parameters
		/////////////////////////
		std::string p_filePath;						//the csv file path

		//////////////////////////
		//members
		/////////////////////////
		std::vector<Vertex2D> m_pointCloud;			//output point cloud of Vertex2D
	};
}
#endif //CSV_DATA_LOADER_HPP
