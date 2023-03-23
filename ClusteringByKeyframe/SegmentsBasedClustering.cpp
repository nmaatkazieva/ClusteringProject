#include "SegmnetsBasedClustering.hpp"

using namespace Clustering;

/////////////////////////////////////////////
//
/////////////////////////////////////////////
SegmentsBasedClustering::SegmentsBasedClustering(const float maxRadius, const float segmentsCount, const float angleTreashold, const float distTreashold)
{
	p_maxRadius = maxRadius;					//set max radius parameter
	p_segmentsCount = segmentsCount;			//set segments count parameter
	p_angleTreshold = angleTreashold;			//set angle treashold parameter
	p_distTreashold = distTreashold;			//set distance treashold parameter
}

/////////////////////////////////////////////
//
/////////////////////////////////////////////
void SegmentsBasedClustering::Process(std::vector<Vertex2D> pointCloud) noexcept
{
	//Sort point cloud by increasing angle 
	std::vector<Vertex2D> soretdPointCloud{ SortPointsByAngle(pointCloud) };		//sorted point cloud

	//Divide data by segments
	std::vector<Segment> segments{ SegmentPointsByRadii(soretdPointCloud) };		//list of segments 

	//Cluster points in each segment
	std::vector<Cluster> segmentedClusters{ ClusterPointsOfSegments(segments) };	//list of segmented clusters

	//Merge clusters
	m_clusters = MergeClusters(segmentedClusters);									//init member of point cloud
}

/////////////////////////////////////////////
//
/////////////////////////////////////////////
std::vector<Cluster> SegmentsBasedClustering::GetData() noexcept
{
	return m_clusters;		//return pointcloud
}


/////////////////////////////////////////////
//
/////////////////////////////////////////////
std::vector<Vertex2D> SegmentsBasedClustering::SortPointsByAngle(std::vector<Vertex2D> pointCloud) noexcept
{
	//Sort points by increasing angle
	for (int i = 0; i < pointCloud.size(); i++)//go through all points in cloud
	{
		pointCloud[i].angleDeg = std::atan2(pointCloud[i].y, pointCloud[i].x) * 180.F / 3.14F;		//calculate angle of point to X axis
		pointCloud[i].distance = std::sqrt(pointCloud[i].x * pointCloud[i].x + pointCloud[i].y * pointCloud[i].y);	//calculate the distance of point to Ego
	}//end for
	//Sort points by increasing angles
	std::sort(pointCloud.begin(), pointCloud.end(), [](const Vertex2D& a, const Vertex2D& b) {return a.angleDeg < b.angleDeg; });
	return pointCloud;	//return sorted point cloud
}

/////////////////////////////////////////////
//
/////////////////////////////////////////////
std::vector<SegmentsBasedClustering::Segment> SegmentsBasedClustering::SegmentPointsByRadii(std::vector<Vertex2D> pointCloud) noexcept//TODO optimize the code
{
	float densityRad{ p_maxRadius / p_segmentsCount };	//the width of each segment
	std::vector<Segment> segments{};					//list of segments
	//create segments list
	for (int i = 0; i < p_segmentsCount; i++)
	{
		Segment s{};							 //create the segment
		s.minRadius = i * densityRad;			 //set min radius
		s.maxRadius = (i + 1) * densityRad;		 //set max radius
		segments.push_back(s);					 //add to segments list
	}//end for

	//go through all points and find each segment's pointcloud
	for (int i = 0; i < pointCloud.size(); i++)
	{
		//go through all segments
		for (int j = 0; j < segments.size(); j++)
		{
			if ((pointCloud[i].distance > segments[j].minRadius) && (pointCloud[i].distance <= segments[j].maxRadius)) //if point is inside the segment
			{
				segments[j].points.push_back(pointCloud[i]);		//add point to the segment's points data
			}//end if
		}//end for
	}
	return segments;		//return segments list
}

/////////////////////////////////////////////
//
/////////////////////////////////////////////
std::vector<Vertex2D> SegmentsBasedClustering::FindBoundaryPointsOfPointCloud(std::vector<Vertex2D> pointCloud) noexcept
{
	std::vector<Vertex2D> pointsToReturn{};						   //list of boundary points of cloud
	std::vector<Vertex2D>::iterator it{};						   //iterator of point cloud
	std::set<int> boundaryPsIndices{};							   //set of boundary points indices

	//Find the most left boundary point
	it = std::min_element(pointCloud.begin(), pointCloud.end(),
		[](const Vertex2D& p1, const Vertex2D& p2) {return p1.angleDeg < p2.angleDeg; });
	boundaryPsIndices.insert(std::distance(pointCloud.begin(), it));

	//Find the most right boundary point
	it = std::max_element(pointCloud.begin(), pointCloud.end(),
		[](const Vertex2D& p1, const Vertex2D& p2) {return p1.angleDeg < p2.angleDeg; });
	boundaryPsIndices.insert(std::distance(pointCloud.begin(), it));

	//Find the closest boundary point
	it = std::min_element(pointCloud.begin(), pointCloud.end(),
		[](const Vertex2D& p1, const Vertex2D& p2) {return p1.distance < p2.distance; });
	boundaryPsIndices.insert(std::distance(pointCloud.begin(), it));

	//Find the farthest boundary point
	it = std::max_element(pointCloud.begin(), pointCloud.end(),
		[](const Vertex2D& p1, const Vertex2D& p2) {return p1.distance < p2.distance; });
	boundaryPsIndices.insert(std::distance(pointCloud.begin(), it));

	//go through set of points indices
	for (const int& j : boundaryPsIndices)
	{
		pointsToReturn.push_back(pointCloud[j]);		//add point to boundary points list
	}//end for
	return pointsToReturn;		//return list of boundary points
}

/////////////////////////////////////////////
//
/////////////////////////////////////////////
float SegmentsBasedClustering::GetDistanceBetweenClusters(const Cluster& c1, const Cluster& c2) noexcept
{
	float minDistance{ std::numeric_limits<float>::max() };//the least distance between cluster's boundary points
	//go through  all boundary points of first cluster
	for (const Vertex2D& p1 : c1.boundaryPs)
	{
		//go through  all boundary points of second cluster
		for (const Vertex2D& p2 : c2.boundaryPs)
		{
			float distance{ std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y)) };	//calculate the euclidian distance between two boundary points
			if (distance < minDistance)	//if the distance is less than calculated before min distance 
			{
				minDistance = distance;		//set min distance
			}//end if
			else
			{
				//min distance remains the same
			}//end else
		}//end for
	}//end for
	return minDistance;		//return min distance
}

/////////////////////////////////////////////
//
/////////////////////////////////////////////
std::vector<Cluster> SegmentsBasedClustering::ClusterPointsOfSegments(std::vector<Segment> segments) noexcept
{
	std::vector<Cluster> segmentedClusters{};					   //list of segmented clusters
	std::map<int, std::vector<Vertex2D>> mapClusters{};			   //map of segmented clusters 
	int clusterId{ 0 };												//cluster id

	//Cluster the points in diffferent segments
	for (const Segment& s : segments)//go through all segments
	{
		if (!s.points.empty())//if segment's pointcloud is not empty
		{
			float prevPointAngle = s.points[0].angleDeg;//previous point's angle
			//Cluster the points inside segment
			for (int j = 0; j < s.points.size(); j++)//go through all points of the segment
			{
				float currentPointAngle = s.points[j].angleDeg;//get the angle of current point
				//Check the differnce between previous and current points angles
				if ((std::fabs(currentPointAngle - prevPointAngle) <= p_angleTreshold))  //if less than treashold angle
				{
					//points are in the same cluster
				}//
				else
				{
					clusterId++;//create new cluster
				}//
				mapClusters[clusterId].push_back(s.points[j]);			 //add point to cluster
				prevPointAngle = currentPointAngle;						 //set the angle as previous point angle
			}//end go through all points of segment
			clusterId++;//crate new cluster
		}//
	}//
	//Copy map to list of clusters to return
	for (int i = 0; i < mapClusters.size(); i++)//go through all map's data
	{
		Cluster cluster{};							  //create cluster
		cluster.clusterId = i;						  //set id
		cluster.points = mapClusters[i];			  //set pointcloud
		segmentedClusters.push_back(cluster);		  //add cluster to list
	}
	//Find boundary points of each cluster
	for (int i = 0; i < segmentedClusters.size(); i++)//go through all segmented clusters
	{
		segmentedClusters[i].boundaryPs = FindBoundaryPointsOfPointCloud(segmentedClusters[i].points);//calculate boundary points
	}
	return segmentedClusters;//return list of segmented clusters
}

/////////////////////////////////////////////
//
/////////////////////////////////////////////
std::vector<Cluster> SegmentsBasedClustering::MergeClusters(std::vector<Cluster> segmentedClusters) noexcept
{
	int i = 0;//index of first segmented cluster
	//Compare the clusters
	while (i < segmentedClusters.size() - 1)//while index of first cluster exists
	{
		int j = i + 1;//next cluster's index
		while (j < segmentedClusters.size())//while index of next cluster exists
		{
			float distance = GetDistanceBetweenClusters(segmentedClusters[i], segmentedClusters[j]);//find the least distance between clusters
			//Check the distance
			if (distance < p_distTreashold)//if less than distance treashold
			{
				segmentedClusters[i].points.insert(segmentedClusters[i].points.end(), segmentedClusters[j].points.begin(), segmentedClusters[j].points.end());					  //merge  points
				segmentedClusters[i].clusterId = std::min(segmentedClusters[i].clusterId, segmentedClusters[j].clusterId);														  //set merged cluster id
				segmentedClusters[i].boundaryPs.insert(segmentedClusters[i].boundaryPs.end(), segmentedClusters[j].boundaryPs.begin(), segmentedClusters[j].boundaryPs.end());	  //merge boundary points
				segmentedClusters[i].boundaryPs = FindBoundaryPointsOfPointCloud(segmentedClusters[i].boundaryPs);																  //find new boundary points
				segmentedClusters.erase(segmentedClusters.begin() + j);																											  //remove merged cluster
			}//
			else
			{
				j++;//raise index to compare first cluster with next 
			}//
		}//end while
		i++;//raise index to check next cluster
	}//end while
	return segmentedClusters;//return merged list of clusters
}