#ifndef CLUSTERING_HPP
#define CLUSTERING_HPP

///Std Includes///
#include <iostream>
#include <vector>
#include <map>
#include <cmath>>
#include <algorithm>
#include <random>
#include <set>

///Clustering includes///
#include "Vertex2D.h"
#include "Cluster.h"

namespace Clustering
{
	class SegmentsBasedClustering final
	{
	public:
		SegmentsBasedClustering();

		struct Segment
		{
			float maxRadius;
			float minRadius;
			std::vector<Vertex2D> points;
			std::vector<Cluster> clusters;

		};

		void Process(std::vector<Vertex2D> pointCloud) noexcept;

		std::vector<Cluster> GetData() noexcept;

	private:

		std::vector<Vertex2D> SortPointsByAngle(std::vector<Vertex2D> pointCloud) noexcept;

		std::vector<Segment> SegmentPointsByRadii(std::vector<Vertex2D> pointCloud) noexcept;

		std::vector<Vertex2D> FindBoundaryPointsOfPointCloud(std::vector<Vertex2D> pointCloud) noexcept;

		float GetDistanceBetweenClusters(const Cluster& c1, const Cluster& c2) noexcept;

		std::vector<Cluster> ClusterPointsOfSegments(std::vector<Segment> segments) noexcept;

		std::vector<Cluster> MergeClusters(std::vector<Cluster> segmentedClusters) noexcept;

		//////////////////////////
		//parameters
		/////////////////////////
		float p_maxRadius;
		int p_segmentsCount;
		float p_angleTreshold;
		float p_distTreashold;

		//////////////////////////
		//members
		/////////////////////////
		std::vector<Cluster> m_clusters;
	};
}



#endif //CLUSTERING_HPP
