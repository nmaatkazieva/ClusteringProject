#include "SegmnetsBasedClustering.hpp"

using namespace Clustering;

/////////////////////////////////////////////
//
/////////////////////////////////////////////
SegmentsBasedClustering::SegmentsBasedClustering()
{
	p_maxRadius = 10.F;
	p_segmentsCount = 10;
	p_angleTreshold = 8.F;
	p_distTreashold = 1.F;
}

/////////////////////////////////////////////
//
/////////////////////////////////////////////
void SegmentsBasedClustering::Process(std::vector<Vertex2D> pointCloud) noexcept
{
	std::vector<Vertex2D> soretdPointCloud = SortPointsByAngle(pointCloud);

	std::vector<Segment> segments = SegmentPointsByRadii(soretdPointCloud);

	std::vector<Cluster> segmentedClusters = ClusterPointsOfSegments(segments);

	m_clusters = MergeClusters(segmentedClusters);
}

/////////////////////////////////////////////
//
/////////////////////////////////////////////
std::vector<Cluster> SegmentsBasedClustering::GetData() noexcept
{
	return m_clusters;
}


/////////////////////////////////////////////
//
/////////////////////////////////////////////
std::vector<Vertex2D> SegmentsBasedClustering::SortPointsByAngle(std::vector<Vertex2D> pointCloud) noexcept
{
	for (int i = 0; i < pointCloud.size(); i++)
	{
		pointCloud[i].angleDeg = std::atan2(pointCloud[i].y, pointCloud[i].x) * 180 / 3.14;
		pointCloud[i].distance = std::sqrt(pointCloud[i].x * pointCloud[i].x + pointCloud[i].y * pointCloud[i].y);
	}
	std::sort(pointCloud.begin(), pointCloud.end(), [](const Vertex2D& a, const Vertex2D& b) {return a.angleDeg < b.angleDeg; });
	return pointCloud;
}

/////////////////////////////////////////////
//
/////////////////////////////////////////////
std::vector<SegmentsBasedClustering::Segment> SegmentsBasedClustering::SegmentPointsByRadii(std::vector<Vertex2D> pointCloud) noexcept//TODO optimize the code
{
	float densityRad = p_maxRadius / p_segmentsCount;
	std::vector<Segment> segments{};
	for (int i = 0; i < p_segmentsCount; i++)
	{
		Segment s{};
		s.minRadius = i * densityRad;
		s.maxRadius = (i + 1) * densityRad;
		segments.push_back(s);
	}

	for (int i = 0; i < pointCloud.size(); i++)
	{
		pointCloud[i].distance = std::sqrt(pointCloud[i].x * pointCloud[i].x + pointCloud[i].y * pointCloud[i].y);
		for (int j = 0; j < segments.size(); j++)
		{
			if ((pointCloud[i].distance > segments[j].minRadius) && (pointCloud[i].distance <= segments[j].maxRadius))
			{
				segments[j].points.push_back(pointCloud[i]);
			}
		}
	}
	return segments;
}

/////////////////////////////////////////////
//
/////////////////////////////////////////////
std::vector<Vertex2D> SegmentsBasedClustering::FindBoundaryPointsOfPointCloud(std::vector<Vertex2D> pointCloud) noexcept
{
	std::vector<Vertex2D> pointsToReturn{};
	std::vector<Vertex2D>::iterator it{};
	std::set<int> boundaryPsIndices{};

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

	for (const int& j : boundaryPsIndices)
	{
		pointsToReturn.push_back(pointCloud[j]);
	}
	return pointsToReturn;
}

/////////////////////////////////////////////
//
/////////////////////////////////////////////
float SegmentsBasedClustering::GetDistanceBetweenClusters(const Cluster& c1, const Cluster& c2) noexcept
{
	float minDistance{ std::numeric_limits<float>::max() };
	for (const Vertex2D& p1 : c1.boundaryPs)
	{
		for (const Vertex2D& p2 : c2.boundaryPs)
		{
			float distance = std::sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
			if (distance < minDistance)
			{
				minDistance = distance;
			}
		}
	}
	return minDistance;
}

/////////////////////////////////////////////
//
/////////////////////////////////////////////
std::vector<Cluster> SegmentsBasedClustering::ClusterPointsOfSegments(std::vector<Segment> segments) noexcept
{
	std::vector<Cluster> segmentedClusters{};
	std::map<int, std::vector<Vertex2D>> mapClusters{};

	//Cluster the points in diffferent segments
	int clusterId = 0;
	for (const Segment& s : segments)
	{
		if (!s.points.empty())
		{
			float prevPointAngle = s.points[0].angleDeg;
			for (int j = 0; j < s.points.size(); j++)
			{
				float currentPointAngle = s.points[j].angleDeg;
				if ((std::fabs(currentPointAngle - prevPointAngle) <= p_angleTreshold))  //TODO checking the last and first points angles diff
				{
				}
				else
				{
					clusterId++;
				}
				mapClusters[clusterId].push_back(s.points[j]);
				prevPointAngle = currentPointAngle;
			}
			clusterId++;
		}
	}
	//Copy map to list of clusters to return
	for (int i = 0; i < mapClusters.size(); i++)
	{
		Cluster cluster{};
		cluster.clusterId = i;
		cluster.points = mapClusters[i];
		segmentedClusters.push_back(cluster);
	}
	for (int i = 0; i < segmentedClusters.size(); i++)
	{
		segmentedClusters[i].boundaryPs = FindBoundaryPointsOfPointCloud(segmentedClusters[i].points);
	}
	return segmentedClusters;
}

/////////////////////////////////////////////
//
/////////////////////////////////////////////
std::vector<Cluster> SegmentsBasedClustering::MergeClusters(std::vector<Cluster> segmentedClusters) noexcept
{
	int i = 0;
	while (i < segmentedClusters.size() - 1)
	{
		int j = i + 1;
		while (j < segmentedClusters.size())
		{
			float distance = GetDistanceBetweenClusters(segmentedClusters[i], segmentedClusters[j]);
			if (distance < p_distTreashold)
			{
				segmentedClusters[i].points.insert(segmentedClusters[i].points.end(), segmentedClusters[j].points.begin(), segmentedClusters[j].points.end());
				segmentedClusters[i].clusterId = std::min(segmentedClusters[i].clusterId, segmentedClusters[j].clusterId);
				segmentedClusters[i].boundaryPs.insert(segmentedClusters[i].boundaryPs.end(), segmentedClusters[j].boundaryPs.begin(), segmentedClusters[j].boundaryPs.end());
				segmentedClusters[i].boundaryPs = FindBoundaryPointsOfPointCloud(segmentedClusters[i].boundaryPs);
				segmentedClusters.erase(segmentedClusters.begin() + j);
			}
			else
			{
				j++;
			}
		}
		i++;
	}
	return segmentedClusters;
}