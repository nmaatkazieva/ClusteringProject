
///Std includes///
#include <iostream>
#include <chrono>

///Clustering includes///
#include "SegmnetsBasedClustering.hpp"
#include "CsvDataLoader.hpp"

//#define RANDOM_POINTCLOUD_SECTION

std::vector < Clustering:: Vertex2D > GenerateRandomPoints(int n) 
{
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<> dis(0, 10);

	std::vector<Clustering::Vertex2D> points(n);
	for (int i = 0; i < n; i++) {
		points[i].x = dis(gen);
		points[i].y = dis(gen);
	}
	return points;
}


int main()
{
	std::vector<Clustering::Vertex2D> points{};

#ifdef RANDOM_POINTCLOUD_SECTION
	 points = GenerateRandomPoints(100000);
#endif //RANDOM_POINTCLOUD_SECTION

	std::shared_ptr<Clustering::SegmentsBasedClustering> clusteringBySegments = std::make_shared<Clustering::SegmentsBasedClustering>();
	std::shared_ptr<Clustering::CsvDataLoader> csvDataLoader = std::make_shared<Clustering::CsvDataLoader>("C:\\Users\\NasiMaatkazievastaff\\source\\repos\\ClusteringByKeyframe\\x64\\Debug\\resources\\point_cloud.csv");

	csvDataLoader->Process();
	points = csvDataLoader->GetData();

	auto start_time = std::chrono::high_resolution_clock::now();

	clusteringBySegments->Process(points);
	std::vector<Clustering::Cluster> clusters = clusteringBySegments->GetData();

	auto end_time = std::chrono::high_resolution_clock::now();
	auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
	std::cout << "Time taken: " << duration << " ms" << std::endl;

	for (int i = 0; i < clusters.size(); i++)
	{
		std::cout << "Cluster " << i << ": " << std::endl;

		for (const Clustering::Vertex2D& p : clusters[i].points)
		{
			std::cout << "    " << p.x << ", " << p.y << std::endl;
		}
	}

	return 0;
}
