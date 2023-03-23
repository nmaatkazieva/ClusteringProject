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
	class CsvDataLoader final
	{
	public:
		CsvDataLoader(std::string filePath);

		void Process() noexcept;

		std::vector<Vertex2D> GetData() noexcept;

	private:

		std::map<std::string, std::vector<float>> ReadCsvData() noexcept;

		void FillPointCloud(std::map<std::string, std::vector<float>> csvData) noexcept;

		//////////////////////////
		//parameters
		/////////////////////////
		std::string p_filePath;

		//////////////////////////
		//members
		/////////////////////////
		std::vector<Vertex2D> m_pointCloud;
	};
}
#endif //CSV_DATA_LOADER_HPP
