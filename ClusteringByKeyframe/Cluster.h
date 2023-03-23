#ifndef CLUSTER_H
#define CLUSTER_H

#include <vector>
#include "Vertex2D.h"

namespace Clustering
{
	struct Cluster
	{
		int clusterId;
		std::vector<Vertex2D> points;
		std::vector<Vertex2D> boundaryPs;
	};
}
#endif //CLUSTER_H
