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
	/// <summary>
	/// Class to cluster point cloud by segments-based method
	/// </summary>
	class SegmentsBasedClustering final
	{
	public:
		/// <summary>
		/// Standard constructor
		/// </summary>
		/// <param name="maxRadius">max radius of area to cluster points [m]</param>
		/// <param name="segmentsCount">the number of segments to divide the clustering area</param>
		/// <param name="angleTreashold">max difference angle between points to cluster them [deg]</param>
		/// <param name="distTreashold">max density between clusters to merge them [m]</param>
		SegmentsBasedClustering(const float maxRadius, const float segmentsCount, const float angleTreashold, const float distTreashold);

		/// <summary>
		/// Struct to hold segment's data
		/// </summary>
		struct Segment
		{
			float maxRadius;					//segment's max radius edge
			float minRadius;					//segment;s min radius edge
			std::vector<Vertex2D> points;		//points data of segment
			std::vector<Cluster> clusters;		//clusters data of segment

		};

		/// <summary>
		/// Main func to run all clustering process
		/// </summary>
		/// <param name="pointCloud"></param>
		/// <returns></returns>
		void Process(std::vector<Vertex2D> pointCloud) noexcept;

		/// <summary>
		/// Func to get list of clusters
		/// </summary>
		/// <returns></returns>
		std::vector<Cluster> GetData() noexcept;

	private:

		/// <summary>
		/// 
		/// </summary>
		/// <param name="pointCloud"></param>
		/// <returns></returns>
		std::vector<Vertex2D> SortPointsByAngle(std::vector<Vertex2D> pointCloud) noexcept;

		/// <summary>
		/// 
		/// </summary>
		/// <param name="pointCloud"></param>
		/// <returns></returns>
		std::vector<Segment> SegmentPointsByRadii(std::vector<Vertex2D> pointCloud) noexcept;

		/// <summary>
		/// 
		/// </summary>
		/// <param name="pointCloud"></param>
		/// <returns></returns>
		std::vector<Vertex2D> FindBoundaryPointsOfPointCloud(std::vector<Vertex2D> pointCloud) noexcept;

		/// <summary>
		/// 
		/// </summary>
		/// <param name="c1"></param>
		/// <param name="c2"></param>
		/// <returns></returns>
		float GetDistanceBetweenClusters(const Cluster& c1, const Cluster& c2) noexcept;

		/// <summary>
		/// 
		/// </summary>
		/// <param name="segments"></param>
		/// <returns></returns>
		std::vector<Cluster> ClusterPointsOfSegments(std::vector<Segment> segments) noexcept;

		/// <summary>
		/// 
		/// </summary>
		/// <param name="segmentedClusters"></param>
		/// <returns></returns>
		std::vector<Cluster> MergeClusters(std::vector<Cluster> segmentedClusters) noexcept;

		//////////////////////////
		//parameters
		/////////////////////////
		float p_maxRadius;							  //max radius of area to cluster points [m]
		int p_segmentsCount;						  //the number of segments to divide the clustering area
		float p_angleTreshold;						  //max difference angle between points to cluster them [deg]
		float p_distTreashold;						  //max density between clusters to merge them [m]

		//////////////////////////
		//members
		/////////////////////////
		std::vector<Cluster> m_clusters;			//output list of clusters
	};
}



#endif //CLUSTERING_HPP
